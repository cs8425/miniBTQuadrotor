/****************************************************************************
 *   $Id:: power_api.h 18 2010-11-23 20:55:31Z nxp12832                     $
 *   Project: NXP LPC11xx software example  
 *
 *   Description:
 *     Power API Header File for NXP LPC11Uxx Device Series 
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#ifdef __cplusplus
 extern "C" {
#endif 

#define PWRROMD_PRESENT

typedef struct _PWRD {
  void (*set_pll)(unsigned int cmd[], unsigned int resp[]);
  void (*set_power)(unsigned int cmd[], unsigned int resp[]);
}  PWRD;

typedef struct _ROM {
#ifdef USBROMD_PRESENT
   USB * pUSBD;
#else
   unsigned p_usbd;
#endif /* USBROMD_PRESENT */
   unsigned p_clib;
   unsigned p_cand;
#ifdef PWRROMD_PRESENT
   PWRD * pPWRD;
#else
   unsigned p_pwrd;
#endif /* PWRROMD_PRESENT */
   unsigned p_dev1;
   unsigned p_dev2;
   unsigned p_dev3;
   unsigned p_dev4; 
}  ROM;

//power setup elated definitions
#define PARAM_DEFAULT           0   //default power settings (voltage regulator, flash interface)
#define PARAM_CPU_EXEC          1   //setup for maximum CPU performance (higher current, more computation)
#define PARAM_EFFICIENCY        2   //balanced setting (power vs CPU performance)
#define PARAM_LOW_CURRENT       3   //lowest active current, lowest CPU performance

#define PARAM_CMD_SUCCESS       0   //power setting successfully found
#define PARAM_INVALID_FREQ      1   //specified freq out of range (=0 or > 50 MHz)
#define PARAM_INVALID_MODE      2   //specified mode not valid (see above for valid)

}




/****************************************************************************
 *   $Id:: power_profiles.h 24 2010-12-17 05:55:55Z nxp12832                $
 *   Project: NXP LPC11xx software example  
 *
 *   Description:
 *     Power API Header File for NXP LPC11Uxx Device Series 
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/

/*
 *  Sample power profiles API wrapper function.
 *
 *  This function will configure the system pll and power configuration using the
 *  power profiles API. This function serves as a SAMPLE implemention only. 
 *
 *  Parmeter 0:     New desired clock frequency in Hz (e.g. 12000000UL for 12Mhz)
 *
 *  Return:         If there was an error, it will return the error code of either API
 *                  function that failed. If successful, it will return 0.
 */
uint32_t config_power_mode(unsigned int power_mode);

/*  Memory used by the Power Profiles driver */
static uint32_t command[4], result[3];

uint32_t config_power_mode(unsigned int power_mode) {
    ROM **rom;
    rom = (ROM **)0x1FFF1FF8;
/*  
 *  1: Call set_power to the desired power mode 
 */

    /****   Call the set_power routine ****/
    command[0] = 48;                      //Current freq in MHz
    command[1] = power_mode;                   //Use the designated power mode
    command[2] = 48;                                    //Change the set_power from previous 48MHz to new mode
    (*rom)->pPWRD->set_power(command, result);          //Apply new power mode
    /**************************************/

    if(result[0])
        return result[0];

    return 0; //SUCCESS
}