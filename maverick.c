#define STATE_START_PULSES      0
#define STATE_FIRST_BIT         1
#define STATE_DATA              2

#define PIN                     23

#define WRITEOUTFILE            0
static const char OUTFILE[] =   "/tmp/maverick.log";

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>
#include <time.h>
#include <sys/time.h>

unsigned int volatile start_pulse_counter = 0, detection_state = 0;
unsigned int volatile last_interrupt_micros = 0, last_interrupt_millis;
unsigned int nibble = 0, fd, data_array_index = 0, data_array[13], shift_value = 0, short_bit = 0, add_1st_bit = 1, current_byte = 0, current_bit = 1, bit_count = 0, ISR_status, save_array[13];
unsigned int probe1 = 0, probe2 = 0;
unsigned int probe1_array[6], probe2_array[6];

// make the quarternary convertion
unsigned int quart(unsigned int param)
{
    param &= 0x0F;
    if (param==0x05)
        return(0);
    if (param==0x06)
        return(1);
    if (param==0x09)
        return(2);
    if (param==0x0A)
        return(3);
}

void outputData(void)
{
    unsigned int i = 0;
    probe1 = probe2 = 0;

    if ((save_array[0] != 0xAA) &&
                    (save_array[1] != 0x99) &&
                    (save_array[2] != 0x95) &&
                    (save_array[3] != 0x59)) {
        return;
    }
    probe2_array[0] = quart(save_array[8] & 0x0F);
    probe2_array[1] = quart(save_array[8] >> 4);
    probe2_array[2] = quart(save_array[7] & 0x0F);
    probe2_array[3] = quart(save_array[7] >> 4);
    probe2_array[4] = quart(save_array[6] & 0x0F);

    probe1_array[0] = quart(save_array[6] >> 4);
    probe1_array[1] = quart(save_array[5] & 0x0F);
    probe1_array[2] = quart(save_array[5] >> 4);
    probe1_array[3] = quart(save_array[4] & 0x0F);
    probe1_array[4] = quart(save_array[4] >> 4);

    for (i = 0; i <= 4; i++) {
        probe1 += probe1_array[i] * (1 << (2 * i));
        probe2 += probe2_array[i] * (1 << (2 * i));
    }

    probe1 -= 532;
    probe2 -= 532;
        
    // Fahrenheit conversion
    //probe1 = (((probe1 * 9)/5) + 32);
    //probe2 = (((probe2 * 9)/5) + 32);
        
    printf("Probe 1:%d\tProbe 2:%d\t@%d\n",probe1,probe2,micros());

    if (WRITEOUTFILE) {
        FILE *of;
        of = fopen(OUTFILE, "a");
        if (!of) {
            printf("File open failed!\n");
        }

        time_t now = time(NULL);
        char buff[20];
        strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&now));
        fprintf(of, "Probe 1: %d* | Probe 2: %d* | %s<br />", probe1, probe2,buff);
        fclose(of);
    }
}

void myInterrupt (void) {
    unsigned int time_since_last = 0;
    unsigned int tsl_micros = 0;
    unsigned int bit_ok = 0, i;
    
    // get the time since last interrupt in milli and micro seconds
    time_since_last = (millis() - last_interrupt_millis);
    tsl_micros = (micros() - last_interrupt_micros);

    // store current interrupt time to calculate time since last (above)
    last_interrupt_micros = micros();
    last_interrupt_millis = millis();

    //here we're attempting to detect the Maverick's preamble - 8x pulses of ~5ms each, spaced at ~250us
    if (detection_state == STATE_START_PULSES) {
        //if last interrupt was seen between 3ms and 7ms ago
        if (((time_since_last > 3) && (time_since_last < 7)) && digitalRead(PIN)) {
            start_pulse_counter++;
            if (start_pulse_counter == 8) {
                //printf("Preamble detected @%d\n", millis());
                start_pulse_counter = 0;
                detection_state = STATE_FIRST_BIT;
            }

            //printf("*TRIGGER* Since last pulse:%dms, Time from start:%dms, Pulse count:%d \n",time_since_last, millis(), start_pulse_counter);
        } else if (tsl_micros > 400) {
            start_pulse_counter = 0;
        }
    } else if (detection_state == STATE_FIRST_BIT && digitalRead(PIN)) {
        detection_state = STATE_DATA;
        current_bit=1;
        current_byte=0;
        shift_value=0;
        data_array_index=0;
        bit_ok=0;
        short_bit=0;
        add_1st_bit = 1;
        bit_count = 1;
        //printf("1");
    }

    if (detection_state == STATE_DATA) {
        // this is a debugging parameter to determine the time between 2 interrupts
        //printf("tsl_micros %d\n", tsl_micros);
        if ((tsl_micros > 90) && (tsl_micros < 390)) {
            if (short_bit == 0) {
                short_bit = 1;
            } else {
                short_bit = 0;
                bit_ok = 1;
                //printf("%d",digitalRead(PIN));
            }
        }

        if ((tsl_micros > 390) && (tsl_micros < 650)) {
            if (short_bit == 1) {
                // expected a short bit and something went wrong
                //start over at getting preamble
                detection_state = STATE_START_PULSES;
                //printf("\n!!!PATTERN FAILURE!!! @%d\n",tsl_micros);
            }
            bit_count++;
            //printf("%d",digitalRead(PIN));
            current_bit=digitalRead(PIN);
            bit_ok = 1;
        }

        if (bit_ok) {
            if (add_1st_bit) {
                current_byte = 0x01;
                shift_value = 1;
                add_1st_bit = 0;
            }

            current_byte = (current_byte << 1) + current_bit;
            shift_value++;
            nibble = current_byte;

            if (shift_value == 8) {
                data_array[data_array_index++] = current_byte;
                //printf("Byte %0d:0x%X @%d (%d)\n", data_array_index, current_byte, micros(), millis());
                bit_count=0;
                shift_value = 0;
                current_byte = 0;
            }

            if (data_array_index == 9) {
                start_pulse_counter = 0;
                //printf("Flushing @%d (%d)\n",micros(), millis());
                detection_state = STATE_START_PULSES;

                for (i=0;i<=9;i++) {
                    save_array[i] = data_array[i];
                }
                //printf("Outputting @%d (%d)\n",micros(), millis());
                outputData();
                //printf("Done @%d (%d)\n", micros(), millis());
            }
            bit_ok = 0;
        }
    }
}

int main(int argc, char **argv) {
    wiringPiSetupSys();
    pinMode (PIN, INPUT);
    printf("Listening for events on pin %d\n",PIN);
    wiringPiISR (PIN, INT_EDGE_BOTH, &myInterrupt);
    // Process priorising function <-- activating this may lead to break the functionality of the program
    //piHiPri(50);
    for (;;) {
        pause();
        //printf("Waiting ... \n"); fflush (stdout);
    }
    return 0;
}
