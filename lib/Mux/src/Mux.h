#define _MUX_H_
#ifdef _MUX_H_

#include <Arduino.h>
#include <SETTINGS.h>

class Mux
{
    public:
        void begin();
        void loop();

        uint16_t readAnalog(byte channel);
        uint8_t readDigital(byte channel);

        /**
         * Scrive il valore specificato nel canale specificato (PWM).
         *
         * @param channel Canale da selezionare; da 0 a 15.
         * @param value Valore da scrivere; 1 (HIGH), 0 (LOW).
         * @param frequency Frequenza PWM; default: 40KHz.
         * @param resolution Risoluzione PWM; default: 8 bit.
         */
        void writeAnalog(byte channel, uint16_t value, uint32_t frequency = 40000, uint8_t resolution = 8);

        /**
         * Scrive il valore specificato nel canale specificato.
         * Tempo per completare: ~10.65us
         *
         * @param channel Canale da selezionare; da 0 a 15.
         * @param value Valore da scrivere; 1 (HIGH), 0 (LOW).
         */
        void writeDigital(byte channel, uint8_t value);

        /**
         * Seleziona un canale del multiplexer con la modalità specificata.
         * Tempo per completare: ~12.48us
         *
         * @param channel Canale da selezionare; da 0 a 15.
         * @param mode Modalità del canale; READ, WRITE.
         */
        void selectChannel(byte channel, uint8_t mode);

        /**
         * Scrive un valore HIGH (1) o LOW (0) nel canale selezionato.
         * Tempo per completare: tipico ~470ns; massimo 1.46us; minimo (one shot, senza altre operazioni) 93ns.
         * @param value Valore da scrivere; 1 (HIGH), 0 (LOW).
         */
        void fastWrite(uint8_t value);

        uint16_t fastRead();

        float requestUSDistance(uint8_t sensor);

        /**
         * @return unsigned long* Puntatore all'array che contiene i dati del multiplexer
         */
        unsigned long* getPacketPointer();

        /**
         * Abilita o disabilita il polling del pacchetto dati dei sensori
         * Frequenza di aggiornamento: ~50Hz
         *
         * US_F, US_L, US_R, IR_F, IR_L, BAT, READ_DIGITAL, READ_ANALOG, PACKET_ID
         */
        void sensPacketUpdate(bool polling_mode);
};

#endif
