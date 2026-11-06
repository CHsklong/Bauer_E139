/**********************************************************************/
// File Name: {FM_project_loc}/../src/projects/e769/m1_step-FHB3915_appconfig.h 
//
// Date:  March 16, 2023, 13:35:36
//
// Automatically generated file for static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __M1_PMSM_APPCONFIG_H
#define __M1_PMSM_APPCONFIG_H
#include "mcdrv.h"


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 4 [-]
//Stator resistance                     = 2.49 [Ohms]
//Direct axis inductance                = 0.00635 [H]
//Quadrature axis inductance            = 0.00635 [H]
//Back-EMF constant                     = 0.297 [V.sec/rad]
//Drive inertia                         = 0.062 [kg.m2]
//Nominal current                       = 3 [A]
#define M1_Rs                                    (2.49F)
#define M1_Ld                                    (0.00635F)
#define M1_Lq                                    (0.00635F)
#define M1_J                                     (0.015F)
#define M1_U_ph                                  (48)
#define M1_N_req                                 (3000)
#define M1_N_min                                 (0)
#define M1_N_max                                 (3600)
#define M1_N_over                                (3800)

#define M1_MOTOR_PP                        (4)
//----------------------------------------------------------------------

//Application scales                    
//----------------------------------------------------------------------
//#define M1_I_MAX                           (9.16F)
//#define M1_I_MAX                           (9.0F)
#define M1_I_MAX                           (8.25F) //16.5
//#define M1_U_DCB_MAX                       (110.0F)
#define M1_U_DCB_MAX                       (71.24F) //71.24
#define M1_U_MAX                           (27.7F)
#define M1_N_MAX                           (1508.0F)
#define M1_FREQ_MAX                        (240.0F)
#define M1_U_DCB_TRIP                      (36.0F)
#define M1_U_DCB_UNDERVOLTAGE              (2.0F)
#define M1_U_DCB_OVERVOLTAGE               (40.0F)
#define M1_N_OVERSPEED                     (1256.6F)
#define M1_N_MIN                           (0.0F)
#define M1_N_NOM                           (1256.6F)
#define M1_I_PH_NOM                        (3.0F)
//DCB Voltage Filter                    
#define M1_UDCB_IIR_B0                     (0.030459027951F)
#define M1_UDCB_IIR_B1                     (0.030459027951F)
#define M1_UDCB_IIR_A1                     (0.939081944097F)
//Mechanical Alignment                  
#define M1_ALIGN_VOLTAGE                   (2.0F)
#define M1_ALIGN_DURATION                  (2000)
//Application counters                  
#define M1_CALIB_DURATION                  (2000)

#define M1_FAULT_DURATION                  (60000)
#define M1_FREEWHEEL_DURATION              (15000)
//Miscellaneous                         
#define M1_E_BLOCK_TRH                     (3.0F)
#define M1_E_BLOCK_PER                     (2000)
#define M1_DRIVER_INERTIA                  (0.015F)
#define M1_BACK_EMF                        (2.1F)

#define M1_OVERLOAD_PER                    (100000)
//Current Loop Control                  
//----------------------------------------------------------------------
//Loop bandwidth                        = 50 [Hz]
//Loop attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Current Controller Output Limit       
#define M1_CLOOP_LIMIT                     (0.461880430704F)
//D-axis Controller - Parallel type     
#define M1_D_KP_GAIN                       (1.499822670059F)
#define M1_D_KI_GAIN                       (0.062671987947F)
//Q-axis Controller - Parallel type     
#define M1_Q_KP_GAIN                       (1.499822670059F)
#define M1_Q_KI_GAIN                       (0.062671987947F)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop bandwidth                        = 1 [Hz]
//Loop attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define M1_SPEED_PI_PROP_GAIN              (0.02F)
#define M1_SPEED_PI_INTEG_GAIN             (0.00002F)
#define M1_SPEED_LOOP_HIGH_LIMIT           (4.0F)
#define M1_SPEED_LOOP_LOW_LIMIT            (-4.0F)
/*
#define M1_SPEED_RAMP_UP                   (2.094395102393F)
#define M1_SPEED_RAMP_DOWN                 (2.094395102393F)
*/
#define M1_SPEED_RAMP_UP                   (100000.0F)
#define M1_SPEED_RAMP_DOWN                 (100000.0F)

/* required speed limitations */         
#define M1_SPEED_REQ_RMP_ACC             (500.0F) //500
#define M1_SPEED_REQ_RMP_JRK             (500.0F) //500
#define M1_SPEED_REQ_RMP_DUR             (0.05F)

#if 0 
#define M1_SPEED_IIR_B0                    (0.135755248164F)
#define M1_SPEED_IIR_B1                    (0.135755248164F)
#define M1_SPEED_IIR_A1                    (0.728489503673F)
#else
#define M1_SPEED_IIR_B0                    (0.239057223611F)
#define M1_SPEED_IIR_B1                    (0.239057223611F)
#define M1_SPEED_IIR_A1                    (0.521885552779F)

#endif
//Position Loop Control                 
//----------------------------------------------------------------------

#define M1_POS_PI_PROP_GAIN                (1.0F)
#define M1_POS_PI_INTEG_GAIN               (0.0F)
#define M1_POS_LOOP_HIGH_LIMIT             (3000.0F)
#define M1_POS_LOOP_LOW_LIMIT              (-3000.0F)

#define M1_POS_RAMP_UP                     (0.0F)
#define M1_POS_RAMP_DOWN                   (0.0F)

/* required pos limitations */
#define M1_POS_VMAX                      (6000.0F)   
#define M1_POS_AMAX                      (2000000.0F)        
#define M1_POS_JMAX                      (4000000.0F) 


#if 0
#define M1_POS_IIR_B0                      (0.024516572555F)
#define M1_POS_IIR_B1                      (0.024516572555F)
#define M1_POS_IIR_A1                      (0.95096685489F)
#else
#define M1_POS_IIR_B0                      (0.239057223611F)
#define M1_POS_IIR_B1                      (0.239057223611F)
#define M1_POS_IIR_A1                      (0.521885552779F)

#endif
//Position & Speed Sensors Module       
//----------------------------------------------------------------------
//Loop Bandwidth                        = 500 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
#define M1_POSPE_TO_KP_GAIN                (6283.185307179586F)
#define M1_POSPE_TO_KI_GAIN                (986.9604401089358F)
#define M1_POSPE_TO_INTEG_GAIN             (0.000031830989F)
#define M1_POSPE_ENC_PULSES                (4096)
#if (BOARD_TYPE == BOARD_E145)
#define M1_POSPE_ENC_DIR                   (1)
#else
#define M1_POSPE_ENC_DIR                   (0)
#endif
#define M1_POSPE_IIR_B0                    FRAC32(0.007732519502)
#define M1_POSPE_IIR_B1                    FRAC32(0.007732519502)
#define M1_POSPE_IIR_A1                    FRAC32(0.484534960997)
#define M1_POSPE_INTEG_GAIN                ACC32(0.048)
#define M1_POSPE_MECH_POS_GAIN             ACC32(4.0)
#define M1_HALL_PLCMNT_NEG_0               FRAC16(-0.53)
#define M1_HALL_PLCMNT_NEG_1               FRAC16(0.13)
#define M1_HALL_PLCMNT_NEG_2               FRAC16(-0.2)
#define M1_HALL_PLCMNT_NEG_3               FRAC16(0.79)
#define M1_HALL_PLCMNT_NEG_4               FRAC16(-0.86)
#define M1_HALL_PLCMNT_NEG_5               FRAC16(0.46)
#define M1_HALL_PLCMNT_POS_0               FRAC16(-0.8)
#define M1_HALL_PLCMNT_POS_1               FRAC16(-0.18)
#define M1_HALL_PLCMNT_POS_2               FRAC16(-0.49)
#define M1_HALL_PLCMNT_POS_3               FRAC16(0.48)
#define M1_HALL_PLCMNT_POS_4               FRAC16(0.76)
#define M1_HALL_PLCMNT_POS_5               FRAC16(0.13)
#define M1_HALL_STATE_ORDER_0              (6)
#define M1_HALL_STATE_ORDER_1              (2)
#define M1_HALL_STATE_ORDER_2              (1)
#define M1_HALL_STATE_ORDER_3              (4)
#define M1_HALL_STATE_ORDER_4              (5)
#define M1_HALL_STATE_ORDER_5              (3)

//Sensorless BEMF DQ nad Tracking Observer
//----------------------------------------------------------------------
//Loop Bandwidth                        = 300 [Hz]
//Loop Attenuation                      
//Loop sample time                      
//----------------------------------------------------------------------
//Bemf DQ Observer                      
#define M1_I_SCALE                         (0.962267010154F)
#define M1_U_SCALE                         (0.015153811184F)
#define M1_E_SCALE                         (0.015153811184F)
#define M1_WI_SCALE                        (0.000096226702F)
#define M1_BEMF_DQ_KP_GAIN                 (21.448936020355F)
#define M1_BEMF_DQ_KI_GAIN                 (2.25619156609F)

//Bemf DQ Observer                      
#define M1_TO_KP_GAIN                      (879.645943005142F)
#define M1_TO_KI_GAIN                      (19.344424626136F)
#define M1_TO_THETA_GAIN                   (0.000031830989F)
//Observer speed output filter          
#define M1_TO_SPEED_IIR_B0                 (0.556862724144F)
#define M1_TO_SPEED_IIR_B1                 (0.556862724144F)
#define M1_TO_SPEED_IIR_A1                 (-0.113725448288F)
//Open loop start-up                    
#define M1_OL_START_RAMP_INC               (0.125663706144F)
#define M1_OL_START_I                      (0.65F)
#define M1_MERG_SPEED_TRH                  (209.43951023932F)
#define M1_MERG_COEFF                      FRAC16(0.004577636719)

//Control Structure Module - Scalar Control
//----------------------------------------------------------------------
#define M1_SCALAR_VHZ_FACTOR_GAIN          (0.24F)
#define M1_SCALAR_INTEG_GAIN               ACC32(0.048)
#define M1_SCALAR_RAMP_UP                  (2.094395102393F)
#define M1_SCALAR_RAMP_DOWN                (2.094395102393F)

#endif

//End of generated file                 
/**********************************************************************/
