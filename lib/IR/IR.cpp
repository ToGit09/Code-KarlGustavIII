#include "IR.h"

IRC IRh;

I2C_bus i2c1(Wire1);
Device_handle ir_ring_handle = {i2c1, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

void IRC::init()
{
    Wire1.begin();
    Wire1.setClock(1000000);
}

int IRC::GetTSSP()
{
    return (int)ir_ring.read_tssp();
}

int IRC::GetTSSP(bool (&tssp)[8])
{
    ir_ring.read_tssp(tssp);

    return (int)ir_ring.read_tssp();
}

void IRC::read(bool readRawData)
{
    uint16_t calib_data[16];
    // ir_ring.read_calibrated_values(calib_data);
    if (readRawData)
    {
        ir_ring.read_raw_values(calib_data);

        // Serial.print("IR: ");

        for (int i = 0; i < 16; i++)
        {
            IR_Values[i] = calib_data[i];
            // Serial.print(IR_Values[i]);
            // Serial.print(" ");
        }

        // Serial.println("");
    }

    Angle = ir_ring.read_ball_angle();

    TSSP = ir_ring.read_tssp();

    Distance = ((ir_ring.read_ball_distance() - 11) / 3);

    Distance_raw = ir_ring.read_ball_distance();

    if (Distance < 0)
    {
        Distance = 0;
    }
}

void IRC::writeGains(uint16_t (&gains)[16])
{
    ir_ring.write_gains(gains);
}

void IRC::writeOffsets(int16_t (&offsets)[16])
{
    ir_ring.write_offsets(offsets);
}

int IRC::GetData(int Port)
{ // raw Data of the IR Sensors
    return IR_Values[Port];
}