#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/clocks.h" //PER SETTARE FREQUENZA CLOCK

#include "pico/multicore.h"
#include "pico/util/queue.h"

#define PLL_SYS_KHZ (125 * 1000)

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define n_mot 4

#define kp 14000.0
#define ki 400.0
#define kd 800.0
#define diametro 69.0

#define Plim 50000.0
#define Ilim 4000.0
#define Dlim 10000.0

//ENCODER
#define ENC_PPR 680.0                               //periodi del canale A per giro ruota
#define ENC_CPR (ENC_PPR * 4.0)                     //fronti in quadratura (A e B, salita e discesa) per giro
#define MM_PER_FRONTE (diametro * M_PI / ENC_CPR)
#define ENC_STOP_US 100000                          //nessun fronte per 100 ms -> ruota ferma

//PWM MOTORI (TB6612FNG: fPWM max 100 kHz)
#define PWM_MOT_WRAP 6249                           //125 MHz / 1 / 6250 = 20 kHz
#define PWM_FULL 50000.0                            //fondo scala uscita PID

//FAILSAFE: motori fermi se il Raspberry non manda niente per questo tempo
#define SERIAL_TIMEOUT_US 3000000

typedef struct {
    volatile uint IN1;
    volatile uint IN2;
    volatile uint PWM;
    volatile uint PWM_CHAN;
    volatile uint slice;
    volatile int corr;
} def_driver;

typedef struct {
    volatile uint IN1;
    volatile uint IN2;
} def_encoder;

typedef struct {
	volatile int32_t count;     //posizione in fronti di quadratura, con segno
	volatile uint32_t t[8];     //istanti (us) degli ultimi fronti, buffer circolare
	volatile uint8_t idx;       //indice dell'ultimo fronte in t[]
	volatile uint8_t n_valid;   //fronti consecutivi nello stesso verso presenti in t[]
	volatile uint8_t state;     //ultimo stato (A<<1)|B
	volatile int8_t dir;        //verso dell'ultimo fronte
	volatile uint32_t errori;   //transizioni non valide (fronte perso)
	int32_t zero;               //count all'ultimo reset distanza
    def_encoder ENC;
} def_readspeed;

typedef struct {
	volatile uint mot;
	volatile double P;
	volatile double I[n_mot];
	volatile double D;
	volatile double correction;
    volatile double old_error[n_mot];
    volatile uint pwm[n_mot];
} def_PID;

typedef struct {
	double trav[4];
	double temp;
} def_dis;

def_driver DVR[n_mot];
def_readspeed RSP[n_mot];
def_PID PID;
def_dis DIS;

int8_t enc_mot[NUM_BANK0_GPIOS]; //gpio -> motore, -1 se non e' un encoder

queue_t speed_q[n_mot];
queue_t dir_q[n_mot];
queue_t pid_q;

queue_t kitd_q;
queue_t kits_q;

//DISTANZA CONDIVISA TRA I CORE
volatile uint32_t dist_decimi = 0; //core0 -> core1, decimi di mm
volatile uint32_t reset_req = 0;   //core1 -> core0, incrementato a ogni richiesta di reset
volatile uint32_t reset_done = 0;  //core0 -> core1, ultima richiesta eseguita

volatile int EN_MOT;
volatile int SERVO_PIN_PWM;
volatile uint SERVO_PWM;
volatile uint slice_ser;
volatile uint serv_0;

volatile uint RX;
volatile uint TX;

//indice (stato_precedente<<2)|stato, stato = (A<<1)|B
//sequenza in avanti 0 -> 2 -> 3 -> 1 -> 0
static const int8_t QUAD_TABLE[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

void enc_interrupt(uint gpio, uint32_t events){
    if(gpio >= NUM_BANK0_GPIOS)return;
    int m = enc_mot[gpio];
    if(m < 0)return;

    uint32_t now = time_us_32();
    uint32_t pins = gpio_get_all();
    uint8_t state = (((pins >> RSP[m].ENC.IN1) & 1) << 1) | ((pins >> RSP[m].ENC.IN2) & 1);
    uint8_t prev = RSP[m].state;

    if(state == prev)return; //fronte gia' contato o rimbalzo
    RSP[m].state = state;

    if((state ^ prev) == 3){ //A e B cambiati insieme: perso un fronte
        RSP[m].count += 2 * RSP[m].dir;
        RSP[m].errori++;
        RSP[m].n_valid = 0;
        return;
    }

    int8_t d = QUAD_TABLE[(prev << 2) | state];
    RSP[m].count += d;

    if(d != RSP[m].dir){ //cambio verso: la finestra per la velocita' riparte
        RSP[m].dir = d;
        RSP[m].n_valid = 0;
    }
    RSP[m].idx = (RSP[m].idx + 1) & 7;
    RSP[m].t[RSP[m].idx] = now;
    if(RSP[m].n_valid < 8)RSP[m].n_valid++;
}

//velocita' in giri/s (modulo)
double enc_speed(int m){
    uint32_t irq = save_and_disable_interrupts();
    uint8_t n = RSP[m].n_valid;
    uint8_t i = RSP[m].idx;
    uint8_t k = n > 4 ? 4 : (n > 0 ? n - 1 : 0); //intervalli disponibili, al massimo un ciclo
    uint32_t t_last = RSP[m].t[i];
    uint32_t t_old = RSP[m].t[(i - k) & 7];
    restore_interrupts(irq);

    uint32_t elapsed = time_us_32() - t_last;
    if((k == 0) || (elapsed > ENC_STOP_US))return 0.0;

    //periodo di un ciclo completo A/B (4 fronti): annulla l'asimmetria tra i fronti
    double periodo = (double)(t_last - t_old) * 4.0 / k;
    //nessun fronte da piu' di un ciclo: la ruota sta rallentando
    if(elapsed > periodo)periodo = elapsed;

    return 1000000.0 / (periodo * ENC_PPR);
}

void var_setup() {
    DVR[0].PWM = 2;
    DVR[1].PWM = 7;
    DVR[2].PWM = 8;
    DVR[3].PWM = 13;

    DVR[0].IN1 = 3;
    DVR[1].IN1 = 5;
    DVR[2].IN1 = 10;
    DVR[3].IN1 = 11;

    DVR[0].IN2 = 4;
    DVR[1].IN2 = 6;
    DVR[2].IN2 = 9;
    DVR[3].IN2 = 12;

    RSP[0].ENC.IN1 = 26;
    RSP[1].ENC.IN1 = 28;
    RSP[2].ENC.IN1 = 18;
    RSP[3].ENC.IN1 = 20;

    RSP[0].ENC.IN2 = 22;
    RSP[1].ENC.IN2 = 27;
    RSP[2].ENC.IN2 = 19;
    RSP[3].ENC.IN2 = 21;

    EN_MOT = 14;
    SERVO_PIN_PWM = 15;
    SERVO_PWM = 0;

    slice_ser = pwm_gpio_to_slice_num(14);
    DVR[0].slice = pwm_gpio_to_slice_num(2);
    DVR[1].slice = pwm_gpio_to_slice_num(6);
    DVR[2].slice = pwm_gpio_to_slice_num(8);
    DVR[3].slice = pwm_gpio_to_slice_num(12);

    DVR[0].PWM_CHAN = 0;
    DVR[1].PWM_CHAN = 1;
    DVR[2].PWM_CHAN = 0;
    DVR[3].PWM_CHAN = 1;

    PID.I[0] = 0;
    PID.I[1] = 0;
    PID.I[2] = 0;
    PID.I[3] = 0;

    for(int i=0;i<NUM_BANK0_GPIOS;i++){
        enc_mot[i] = -1;
    }
    for(int i=0;i<4;i++){
        enc_mot[RSP[i].ENC.IN1] = i;
        enc_mot[RSP[i].ENC.IN2] = i;
        RSP[i].count = 0;
        RSP[i].zero = 0;
        RSP[i].n_valid = 0;
        RSP[i].dir = 0;
    }

    DVR[0].corr = -1;
    DVR[1].corr = -1;
    DVR[2].corr = -1;
    DVR[3].corr = -1;

    TX = 0;
    RX = 1;

	serv_0 = 1875;

}

void driver_set(int mot, int8_t dir, double pwm) {
    uint32_t mask = (1u << DVR[mot].IN1) | (1u << DVR[mot].IN2);
    uint level;

    dir = dir * DVR[mot].corr;
    if (dir == 1){ //CW: IN1=H IN2=L, PWM=L -> short brake
        gpio_put_masked(mask, 1u << DVR[mot].IN1);
        level = pwm * (PWM_MOT_WRAP + 1) / PWM_FULL;
    }else if (dir == -1){ //CCW: IN1=L IN2=H, PWM=L -> short brake
        gpio_put_masked(mask, 1u << DVR[mot].IN2);
        level = pwm * (PWM_MOT_WRAP + 1) / PWM_FULL;
    }else if (dir == 0){ //FRENA: IN1=IN2=H -> short brake con qualsiasi PWM
        gpio_put_masked(mask, mask);
        level = 0;
    }else{ //OFF: IN1=IN2=L con PWM=H -> uscite in alta impedenza
        gpio_put_masked(mask, 0);
        level = PWM_MOT_WRAP + 1;
    }
    pwm_set_chan_level(DVR[mot].slice, DVR[mot].PWM_CHAN, level);
}

void mot_setup() {
    //GPIO + PWM
    gpio_set_function(SERVO_PIN_PWM, GPIO_FUNC_PWM);

    gpio_init(EN_MOT);
    gpio_set_dir(EN_MOT, GPIO_OUT);
    gpio_put(EN_MOT, 0);

    for(int i=0;i<4;i++){
        gpio_set_function(DVR[i].PWM, GPIO_FUNC_PWM);

        gpio_init(DVR[i].IN1);
        gpio_set_dir(DVR[i].IN1, GPIO_OUT);
        gpio_put(DVR[i].IN1, 0);

        gpio_init(DVR[i].IN2);
        gpio_set_dir(DVR[i].IN2, GPIO_OUT);
        gpio_put(DVR[i].IN2, 0);
    }

    pwm_set_clkdiv(slice_ser, 100.0);
    pwm_set_wrap(slice_ser, 25000);
    pwm_set_chan_level(slice_ser, PWM_CHAN_B, serv_0);
    pwm_set_enabled(slice_ser, true);

    for(int i=0;i<4;i++){
        pwm_set_clkdiv(DVR[i].slice, 1.0);
        pwm_set_wrap(DVR[i].slice, PWM_MOT_WRAP);
        driver_set(i, 2, 0);
        pwm_set_enabled(DVR[i].slice, true);
    }

    //STBY ALTO SOLO CON I DRIVER GIA' IN UNO STATO DEFINITO
    gpio_put(EN_MOT, 1);

    //ENCODER IN QUADRATURA: INTERRUPT SU ENTRAMBI I FRONTI DI ENTRAMBI I CANALI
    for(int i=0;i<4;i++){
        gpio_init(RSP[i].ENC.IN1);
        gpio_init(RSP[i].ENC.IN2);
        RSP[i].state = (gpio_get(RSP[i].ENC.IN1) << 1) | gpio_get(RSP[i].ENC.IN2);
    }

    uint32_t fronti = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(RSP[0].ENC.IN1, fronti, true, &enc_interrupt);
    for(int i=0;i<4;i++){
        gpio_set_irq_enabled(RSP[i].ENC.IN1, fronti, true);
        gpio_set_irq_enabled(RSP[i].ENC.IN2, fronti, true);
    }
}

void uart_setup() {
    uart_init(uart0, 1000000);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);
    uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);

    uart_set_hw_flow(uart0, false, false);
    uart_set_fifo_enabled(uart0, true);
}

uint serv_getduty(int ang) {
    int duty = ((ang / 18.0) * 125.0) + 1875;
    return duty;
}

uint8_t kitd = 0;
uint8_t gkitd = 0;
uint8_t kits = 0;
uint8_t gkits = 0;
bool kit(struct repeating_timer *t) {

    gkitd = 0;
    gkits = 0;
    queue_try_remove(&kitd_q, &gkitd);
    kitd += gkitd;
    queue_try_remove(&kits_q, &gkits);
    kits += gkits;

    if(kitd > 0){
        if(SERVO_PWM == serv_0){
            SERVO_PWM = SERVO_PWM = serv_getduty(180);
            kitd--;
        }else{
            SERVO_PWM = serv_0;
        }
    }else if(kits > 0){
        if(SERVO_PWM == serv_0){
            SERVO_PWM = SERVO_PWM = serv_getduty(-180);
            kits--;
        }else{
            SERVO_PWM = serv_0;
        }
    }else{
        SERVO_PWM = serv_0;
    }

    pwm_set_chan_level(slice_ser, PWM_CHAN_B, SERVO_PWM);

    return true;
}

bool pid(struct repeating_timer *t) {
    uint8_t pidavv = 0;
    queue_try_add(&pid_q, &pidavv); //se il loop e' in ritardo il tick viene saltato
    return true;
}

//velocita' e verso per una coppia di motori (primo = 0 o 2)
void motori_set(int primo, double velocita, int8_t direzione) {
    double sus;
    int8_t sus1;

    queue_try_remove(&speed_q[primo], &sus);
    queue_try_remove(&speed_q[primo+1], &sus);
    queue_try_remove(&dir_q[primo], &sus1);
    queue_try_remove(&dir_q[primo+1], &sus1);

    queue_try_add(&speed_q[primo], &velocita);
    queue_try_add(&speed_q[primo+1], &velocita);
    queue_try_add(&dir_q[primo], &direzione);
    queue_try_add(&dir_q[primo+1], &direzione);
}

void main_1() {

    uint8_t buf, vel;
    uint8_t byte[8];
    uint8_t check = false;
    uint32_t ultimo_rx = time_us_32();
    bool fermo = false;

    while(1){

        check = uart_is_readable_within_us(uart0, 2000);

        if(check == false){

            //FAILSAFE: NESSUN DATO DAL RASPBERRY -> FRENA TUTTI I MOTORI
            if(!fermo && (time_us_32() - ultimo_rx > SERIAL_TIMEOUT_US)){
                motori_set(0, 0.0, 0);
                motori_set(2, 0.0, 0);
                fermo = true;
            }

        }else{ //SERIALE LEGGIBILE

            ultimo_rx = time_us_32();
            fermo = false;

            buf = 0;
            uart_read_blocking(uart0, &buf, 1);
            for(int i=0;i<8;i++){
                byte[i] = (buf >> i) & 1;
            }

            if(byte[7] == 0){

                if((byte[6] == 0) && (byte[5] == 1)){ //SET MOTORI

                    check = uart_is_readable_within_us(uart0, 2000);

                    if(check == true){

                        uart_read_blocking(uart0, &buf, 1);
                        vel = buf;
                        buf = (buf >> 7) & 1;

                        if(buf == 1){

                            double velocita = (vel & 127) / 50.0;
                            int8_t direzione;
                            int8_t sx;

                            if((byte[1] == 1) && (byte[0] == 1)){ //FRENA
                                direzione = 0;
                            }else if((byte[1] == 0) && (byte[0] == 1)){
                                direzione = 1;
                            }else if((byte[1] == 1) && (byte[0] == 0)){
                                direzione = -1;
                            }else{ //OFF
                                direzione = 2;
                            }

                            if(byte[2] == 0){
                                sx = 0;
                            }else{
                                sx = 2;
                            }

                            motori_set(sx, velocita, direzione);

                        }

                    }

                }else if((byte[6] == 1) && (byte[5] == 0)){ //RICHIESTE

                    if((byte[4] == 0) && (byte[3] == 0)){ //RESET DISTANZA

                        reset_req++;

                    }else if((byte[4] == 0) && (byte[3] == 1)){ //MANDA DISTANZA

                        uint32_t distanza = 0; //reset richiesto ma non ancora eseguito -> 0
                        if(reset_done == reset_req){
                            __dmb();
                            distanza = dist_decimi;
                        }
                        if(distanza > 0xFFFF)distanza = 0xFFFF;

                        uint8_t temp;
                        uint16_t data = distanza;

                        temp = data & 0xFF;
                        uart_putc_raw(uart0, temp);
                        temp = data >> 8;
                        uart_putc_raw(uart0, temp);

                    }else if((byte[4] == 1) && (byte[3] == 0)){ //KIT

                        uint8_t kit;

                        kit = byte[1] * 2 + byte[0];

                        buf = 0;
                        if(byte[2] == 1){
                            queue_try_remove(&kitd_q, &buf);
                            kit += buf;
                            queue_try_add(&kitd_q, &kit);
                        }else{
                            queue_try_remove(&kits_q, &buf);
                            kit += buf;
                            queue_try_add(&kits_q, &kit);
                        }

                    }

                }

            }

        }

    }

}

int main() {
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    stdio_init_all();

    var_setup();

    for(int i=0;i<4;i++){
        queue_init(&speed_q[i], sizeof(double), 1);
        queue_init(&dir_q[i], sizeof(int8_t), 1);
    }
    queue_init(&kitd_q, sizeof(uint8_t), 1);
    queue_init(&kits_q, sizeof(uint8_t), 1);
    queue_init(&pid_q, sizeof(uint8_t), 1);

    mot_setup();
    uart_setup();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    //CORE1 E TIMER SOLO DOPO AVER INIZIALIZZATO TUTTO
    multicore_launch_core1(main_1);

    struct repeating_timer timerpid;
    add_repeating_timer_us(-500, pid, NULL, &timerpid);

    struct repeating_timer timerkit;
    add_repeating_timer_ms(500, kit, NULL, &timerkit);

    double speedtemp;
    double speed[n_mot] = {0};
    double speed_error = 0;
    double wanted_speed[n_mot] = {0};
    int8_t wanted_dir[n_mot] = {0};
    int8_t last_dir[n_mot] = {0};
    bool pid_init[n_mot] = {true, true, true, true};
    uint8_t start;

    for(int i=0;i<4;i++){
        wanted_speed[i] = 0.0;
        speed[i] = 0.0;
        DIS.trav[i] = 0.0;
    }

    while(1) {

        queue_remove_blocking(&pid_q, &start);

        PID.mot ++;
        if (PID.mot >= 4) {
            PID.mot = 0;
        }

        queue_try_remove(&speed_q[PID.mot], &wanted_speed[PID.mot]);
        queue_try_remove(&dir_q[PID.mot], &wanted_dir[PID.mot]);

        speedtemp = enc_speed(PID.mot);

        if (speedtemp < 3.0) { //scarta letture non plausibili
            speed[PID.mot] = speedtemp;
        }

        //RESET PID A MOTORE FERMO O AL CAMBIO DI VERSO
        if ((wanted_speed[PID.mot] == 0) || (wanted_dir[PID.mot] != last_dir[PID.mot])) {
            PID.I[PID.mot] = 0;
            PID.pwm[PID.mot] = 0;
            pid_init[PID.mot] = true;
        }
        last_dir[PID.mot] = wanted_dir[PID.mot];

        PID.P = 0;
        PID.D = 0;
        PID.correction = 0;

        if (wanted_speed[PID.mot]  != 0) {
            speed_error = wanted_speed[PID.mot] - speed[PID.mot];

            if (pid_init[PID.mot]) { //primo campione: niente salto del termine D
                PID.old_error[PID.mot] = speed_error;
                pid_init[PID.mot] = false;
            }

            PID.P = speed_error * kp;
            PID.I[PID.mot] += speed_error * ki;
            PID.D = ((speed_error - PID.old_error[PID.mot]) / 0.002)* kd;

            PID.old_error[PID.mot] = speed_error;

            PID.P = PID.P > Plim ? Plim : PID.P < -Plim ? -Plim : PID.P;
            PID.I[PID.mot] = PID.I[PID.mot] > Ilim ? Ilim : PID.I[PID.mot] < -Ilim ? -Ilim : PID.I[PID.mot];
            PID.D = PID.D > Dlim ? Dlim : PID.D < -Dlim ? -Dlim : PID.D;

            PID.correction = PID.P + PID.I[PID.mot] + PID.D + PID.pwm[PID.mot];

            PID.correction = PID.correction > PWM_FULL ? PWM_FULL : PID.correction < 0 ? 0 : PID.correction;
            PID.pwm[PID.mot] = PID.correction;
        }else{
            PID.pwm[PID.mot] = 0;
        }
        driver_set(PID.mot, wanted_dir[PID.mot], PID.correction);

        //RESET DISTANZA RICHIESTO DA CORE1
        uint32_t req = reset_req;
        if (req != reset_done) {
            for(int i=0;i<4;i++){
                RSP[i].zero = RSP[i].count;
            }
            dist_decimi = 0;
            __dmb();
            reset_done = req;
        }

        //DISTANZA: SPOSTAMENTO NETTO DI OGNI RUOTA, LA MINORE DELLE 4
        DIS.temp = 1000000.0;
        for(int i=0;i<4;i++){
            DIS.trav[i] = abs(RSP[i].count - RSP[i].zero) * MM_PER_FRONTE;
            if (DIS.trav[i] <  DIS.temp)DIS.temp = DIS.trav[i];
        }

        if(DIS.temp>290){
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        } else{
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
        dist_decimi = DIS.temp * 10.0;
    }
}
