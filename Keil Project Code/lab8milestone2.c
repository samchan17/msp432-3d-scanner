/*  COMPENG 2DX3 Final Project – Deliverable 2
    This program implements a rotating time-of-flight sensing system that
    scans the surrounding environment over 360° using a stepper motor.
    Distance measurements are collected at 11.25° increments, processed
    into Cartesian coordinates, and transmitted via UART to MATLAB for
    real-time visualization of the scanned space.

    Written by: Sam Chan (chans134, 400575501)
*/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// I2C control bit masks (taken from Studio 7C (12C))
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

// Project constants
#define SENSOR_ADDRESS          0x29
#define STEP_DELAY_CW           50000
#define STEP_DELAY_CCW          30000
#define STEPS_PER_1125_DEG      16
#define TOTAL_SCAN_ANGLES       32
#define TOTAL_SCANS             3
#define ANGLE_INCREMENT_DEG     11.25f
#define SCAN_SPACING_MM         200.0f
#define PI_VALUE                3.14159265358979f

uint16_t dev = SENSOR_ADDRESS;
int status = 0;

// Raw scan data + converted coordinates
float angle_deg[TOTAL_SCANS][TOTAL_SCAN_ANGLES];
uint16_t distance_mm[TOTAL_SCANS][TOTAL_SCAN_ANGLES];
float angle_rad[TOTAL_SCANS][TOTAL_SCAN_ANGLES];
float x_coord[TOTAL_SCANS][TOTAL_SCAN_ANGLES];
float y_coord[TOTAL_SCANS][TOTAL_SCAN_ANGLES];
float z_coord[TOTAL_SCANS][TOTAL_SCAN_ANGLES];

// -------------------- Hardware Init --------------------

void I2C_Init(void){
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){}

    GPIO_PORTB_AFSEL_R |= 0x0C;
    GPIO_PORTB_ODR_R   |= 0x08;
    GPIO_PORTB_DEN_R   |= 0x0C;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;

    I2C0_MCR_R  = I2C_MCR_MFE;
    I2C0_MTPR_R = 0b0000000000000101000000000111011;
}

void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){}

    GPIO_PORTG_DIR_R &= ~0x01;
    GPIO_PORTG_AFSEL_R &= ~0x01;
    GPIO_PORTG_DEN_R |= 0x01;
    GPIO_PORTG_AMSEL_R &= ~0x01;
}

void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){}

    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
    GPIO_PORTH_DATA_R &= ~0x0F;
}

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){}

    GPIO_PORTJ_DIR_R &= ~0x01;
    GPIO_PORTJ_DEN_R |= 0x01;
    GPIO_PORTJ_PUR_R |= 0x01;
    GPIO_PORTJ_AFSEL_R &= ~0x01;
    GPIO_PORTJ_AMSEL_R &= ~0x01;
}

// GPIO port M to prove bus speed (PM0)
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;   // enable clock for Port M
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}

    GPIO_PORTM_DIR_R |= 0x01;    // PM0 output
    GPIO_PORTM_AFSEL_R &= ~0x01; 
    GPIO_PORTM_DEN_R |= 0x01;    
    GPIO_PORTM_AMSEL_R &= ~0x01; 
}

void Hardware_Init(void){
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortG_Init();
    PortH_Init();
    PortJ_Init();
		PortM_Init();
}

// -------------------- Sensor + Button --------------------

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;
    GPIO_PORTG_DATA_R &= ~0x01;
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;
}

void WaitForButtonPress(void){
    while((GPIO_PORTJ_DATA_R & 0x01) != 0){}
    SysTick_Wait10ms(2);
    while((GPIO_PORTJ_DATA_R & 0x01) == 0){}
    SysTick_Wait10ms(2);
}

void WaitForSensorBoot(void){
    uint8_t sensorState = 0;

    while(sensorState == 0){
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }

    FlashAllLEDs();
}

void Sensor_StartRanging(void){
    status = VL53L1X_ClearInterrupt(dev);
    status = VL53L1X_SensorInit(dev);
    status = VL53L1X_StartRanging(dev);
}

void Sensor_StopRanging(void){
    status = VL53L1X_StopRanging(dev);
}

uint16_t ReadDistanceMeasurement(void){
    uint8_t dataReady = 0;
    uint16_t distance = 0;

    GPIO_PORTN_DATA_R |= 0x01;   // PN0 measurement LED (LED D2)

    while(dataReady == 0){
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        VL53L1_WaitMs(dev, 5);
    }

    status = VL53L1X_GetDistance(dev, &distance);
    status = VL53L1X_ClearInterrupt(dev);

    GPIO_PORTN_DATA_R &= ~0x01;

    return distance;
}

// -------------------- Stepper Motor --------------------

void stepEventCW(uint32_t delay){
    GPIO_PORTH_DATA_R = 0b0011;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b0110;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b1100;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b1001;
    SysTick_Wait(delay);
}

void stepEventCCW(uint32_t delay){
    GPIO_PORTH_DATA_R = 0b1001;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b1100;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b0110;
    SysTick_Wait(delay);
    GPIO_PORTH_DATA_R = 0b0011;
    SysTick_Wait(delay);
}

void Stepper_1125Deg_CW(void){
    int i;
    for(i = 0; i < STEPS_PER_1125_DEG; i++){
        stepEventCW(STEP_DELAY_CW);
    }
}

void Stepper_1125Deg_CCW(void){
    int i;
    for(i = 0; i < STEPS_PER_1125_DEG; i++){
        stepEventCCW(STEP_DELAY_CCW);
    }
}

void ReturnToStartPosition(void){
    int i;

    for(i = 0; i < TOTAL_SCAN_ANGLES; i++){
        Stepper_1125Deg_CCW();
        SysTick_Wait10ms(2);
    }

    GPIO_PORTH_DATA_R = 0x00;
}

// -------------------- Math --------------------

float DegToRad(float degrees){
    return degrees * PI_VALUE / 180.0f;
}

float PolarToX(float radius, float theta_rad){
    return radius * cosf(theta_rad);
}

float PolarToY(float radius, float theta_rad){
    return radius * sinf(theta_rad);
}

void SaveScanPoint(int scan, int index, float angle, uint16_t distance){
    angle_deg[scan][index] = angle;
    distance_mm[scan][index] = distance;
}

void ConvertScanToCartesian(void){
    int scan, i;

    for(scan = 0; scan < TOTAL_SCANS; scan++){
        for(i = 0; i < TOTAL_SCAN_ANGLES; i++){
            angle_rad[scan][i] = DegToRad(angle_deg[scan][i]);
            x_coord[scan][i] = PolarToX((float)distance_mm[scan][i], angle_rad[scan][i]);
            y_coord[scan][i] = PolarToY((float)distance_mm[scan][i], angle_rad[scan][i]);
            z_coord[scan][i] = scan * SCAN_SPACING_MM;   // 0, 100, 200 mm
        }
    }
}

// -------------------- Scan --------------------

void PerformSingleScanStep(int scan, int index){
    float angle;
    uint16_t distance;
		//char buffer[64];

    angle = index * ANGLE_INCREMENT_DEG;
    distance = ReadDistanceMeasurement();
    SaveScanPoint(scan, index, angle, distance);
		
    // 32 readings and 32 CW moves
    Stepper_1125Deg_CW();
    SysTick_Wait10ms(2);
}

void PerformFullScan(int scan){
    int index;

    GPIO_PORTF_DATA_R |= 0x10;   // PF4 scan LED (LED D3)

    for(index = 0; index < TOTAL_SCAN_ANGLES; index++){
        PerformSingleScanStep(scan, index);
    }
		
		SysTick_Wait(14000000);
    GPIO_PORTF_DATA_R &= ~0x10;
}

// -------------------- UART / MATLAB Handshake --------------------

void WaitForMatlabAck(void){
    char c;

    while(1){
        c = UART_InChar();
        if(c == 's'){
            break;
        }
    }
}

void SendReadySignal(void){
    UART_OutChar('s');
}

// Integer-only send functions to avoid float-format UART issues
void SendXData(void){
    int scan, i;
    char buffer[32];
    int x_int;

    GPIO_PORTF_DATA_R |= 0x01;   // PF0 UART LED (LED D4)

    for(scan = 0; scan < TOTAL_SCANS; scan++){
        for(i = 0; i < TOTAL_SCAN_ANGLES; i++){
            x_int = (int)x_coord[scan][i];
            sprintf(buffer, "%d\r\n", x_int);
            UART_printf(buffer);
        }
    }

    GPIO_PORTF_DATA_R &= ~0x01;
}

void SendYData(void){
    int scan, i;
    char buffer[32];
    int y_int;

    GPIO_PORTF_DATA_R |= 0x01;

    for(scan = 0; scan < TOTAL_SCANS; scan++){
        for(i = 0; i < TOTAL_SCAN_ANGLES; i++){
            y_int = (int)y_coord[scan][i];
            sprintf(buffer, "%d\r\n", y_int);
            UART_printf(buffer);
        }
    }

    GPIO_PORTF_DATA_R &= ~0x01;
}

void SendZData(void){
    int scan, i;
    char buffer[32];
    int z_int;

    GPIO_PORTF_DATA_R |= 0x01;

    for(scan = 0; scan < TOTAL_SCANS; scan++){
        for(i = 0; i < TOTAL_SCAN_ANGLES; i++){
            z_int = (int)z_coord[scan][i];
            sprintf(buffer, "%d\r\n", z_int);
            UART_printf(buffer);
        }
    }

    GPIO_PORTF_DATA_R &= ~0x01;
}

// -------------------- Main --------------------

int main(void){
    int scan;

    Hardware_Init();
    WaitForSensorBoot();
    Sensor_StartRanging();
	
		/* AD3 Proving bus speed function (comment out when needed)
		while(1){
			GPIO_PORTM_DATA_R ^= 0x01;
			SysTick_Wait(14000000);
		}
		*/
	

		WaitForButtonPress(); // for interview, one press to start the 3 scans
	
    for(scan = 0; scan < TOTAL_SCANS; scan++){     
        PerformFullScan(scan);
        ReturnToStartPosition();
    }
		
		UART_printf("All 3 scans complete.\r\n");
    ConvertScanToCartesian();
		
    SendReadySignal();
    WaitForMatlabAck();
    SendXData();
		
    SendReadySignal();
    WaitForMatlabAck();
    SendYData();

    Sensor_StopRanging();

    while(1){
    }
}