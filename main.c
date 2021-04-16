/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Littlefs File System on SD Card
*              and QSPI NOR Flash code example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "lfs.h"
#include "lfs_sd_bd.h"
#include "lfs_spi_flash_bd.h"

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* When set to 1, SD card will be used instead of the NOR flash */
#define STORAGE_DEVICE_SD_CARD              (0)

#define LITTLEFS_TASK_STACK_SIZE            (512U)
#define USER_BUTTON_INTERRUPT_PRIORITY      (7U)

/* Debounce delay for the user button. */
#define DEBOUNCE_DELAY_MS                   (50U)

#if (STORAGE_DEVICE_SD_CARD) && \
    (defined (TARGET_CY8CKIT_062_BLE) || \
     defined (TARGET_CY8CKIT_062_WIFI_BT) || \
     defined (TARGET_CY8CPROTO_062S3_4343W) || \
     defined (TARGET_CYW9P62S1_43438EVB_01))
#error The selected target does not support SD card.
#endif /* #if STORAGE_DEVICE_SD_CARD */


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static TaskHandle_t littlefs_task_handle;

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;


/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status (error condition) by
*  turning the LED on, and asserts the non-zero status.
*
* Parameters: 
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
static void check_status(char *message, uint32_t status)
{
    if (0u != status)
    {
        printf("\n================================================================================\n");
        printf("\nFAIL: %s\n", message);
        printf("Error Code: 0x%08"PRIx32"\n", status);
        printf("\n================================================================================\n");
        
        while(true);
    }
}

/*******************************************************************************
* Function Name: print_block_device_parameters
********************************************************************************
* Summary:
*   Prints the block device parameters such as the block count, block size, and
*   program (page) size to the UART terminal.
*
* Parameters:
*  lfs_cfg - pointer to the lfs_config structure.
*
*******************************************************************************/
static void print_block_device_parameters(struct lfs_config *lfs_cfg)
{
    printf("Number of blocks: %"PRIu32"\n", lfs_cfg->block_count);
    printf("Erase block size: %"PRIu32" bytes\n", lfs_cfg->block_size);
    printf("Prog size: %"PRIu32" bytes\n\n", lfs_cfg->prog_size);
}

/*******************************************************************************
* Function Name: user_button_interrupt_handler
********************************************************************************
* Summary:
*   User button interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void user_button_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    (void) handler_arg;
    (void) event;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(littlefs_task_handle, &higher_priority_task_woken);

    /* Yield if xHigherPriorityTaskWoken was set to true */
    portYIELD_FROM_ISR( higher_priority_task_woken );
}

/*******************************************************************************
* Function Name: increment_boot_count
********************************************************************************
* Summary:
*   Mounts the filesystem in the memory and performs basic file I/O operations.
*   And then reads a 32-bit value from a file, increments the value, writes
*   the value back to the file in the memory, and finally prints the value to the
*   UART terminal.
*
* Parameters:
*  lfs - pointer to the lfs_t structure.
*  lfs_cfg - pointer to the lfs_config structure.
*
*******************************************************************************/
static void increment_boot_count(lfs_t *lfs, struct lfs_config *lfs_cfg)
{
    uint32_t boot_count = 0;
    lfs_file_t file;

    /* Mount the filesystem */
    int err = lfs_mount(lfs, lfs_cfg);

    /* Reformat if we cannot mount the filesystem.
     * This should only happen when littlefs is set up on the storage device for
     * the first time.
     */
    if (err) {
        printf("\nError in mounting. This could be the first time littlefs is used on the storage device.\n");
        printf("Formatting the block device...\n\n");

        lfs_format(lfs, lfs_cfg);
        lfs_mount(lfs, lfs_cfg);
    }

    /* Read the current boot count. */
    lfs_file_open(lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(lfs, &file, &boot_count, sizeof(boot_count));

    /* Update the boot count. */
    boot_count += 1;
    lfs_file_rewind(lfs, &file);
    lfs_file_write(lfs, &file, &boot_count, sizeof(boot_count));

    /* The storage is not updated until the file_sd is closed successfully. */
    lfs_file_close(lfs, &file);

    /* Release any resources we were using. */
    lfs_unmount(lfs);

    /* Print the boot count. */
    printf("boot_count: %"PRIu32"\n\n", boot_count);
}

/*******************************************************************************
* Function Name: littlefs_task
********************************************************************************
* Summary:
*   Initializes the block device, prints the block device parameters to the UART
*   terminal, calls the function that performs simple file I/O operations, and
*   waits for user button press. When the user button is pressed, formats the
*   memory and deinitializes the block device.
*
* Parameters:
*  arg - Unused.
*
*******************************************************************************/
static void littlefs_task(void* arg)
{
    cy_rslt_t result;
    lfs_t lfs;
    struct lfs_config lfs_cfg;

    /* Step 1: Get the default configuration for the block device.
     * Step 2: Initialize the lfs_config structure to zero (not required if it
     *         is a global variable)
     * Step 3: Create the block device
     * Step 4: Print the block device parameters such as erase block size
     * Step 5: Perform file system operations to increment the boot count
     */

#if(STORAGE_DEVICE_SD_CARD)
    lfs_sd_bd_config_t sd_bd_cfg;

    printf("Incrementing the boot count on SD Card...\n\n");

    /* Get the default configuration for the SD card block device. */
    lfs_sd_bd_get_default_config(&sd_bd_cfg);

    /* Initialize the pointers in lfs_cfg to NULL. */
    memset(&lfs_cfg, 0, sizeof(lfs_cfg));

    /* Create the SD card block device. */
    result = lfs_sd_bd_create(&lfs_cfg, &sd_bd_cfg);
    check_status("Creating SD card block device failed", result);
#else
    lfs_spi_flash_bd_config_t spi_flash_bd_cfg;

    printf("\nIncrementing the boot count on SPI flash...\n\n");

    /* Get the default configuration for the SPI flash block device. */
    lfs_spi_flash_bd_get_default_config(&spi_flash_bd_cfg);

    /* Initialize the pointers in lfs_cfg to NULL. */
    memset(&lfs_cfg, 0, sizeof(lfs_cfg));

    /* Create the SPI flash block device. */
    result = lfs_spi_flash_bd_create(&lfs_cfg, &spi_flash_bd_cfg);
    check_status("Creating SPI flash block device failed", result);
#endif /* #if(STORAGE_DEVICE_SD_CARD) */

    print_block_device_parameters(&lfs_cfg);
    increment_boot_count(&lfs, &lfs_cfg);

    /* Enable the user button interrupt */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BUTTON_INTERRUPT_PRIORITY, true);

    printf("Press the user button to format the block device or press reset to increment the boot count again\n\n");

    /* Wait until the user button press is notified through the interrupt */
    while (true)
    {
        if(1lu == ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            /* Debounce the button press. */
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));

            if(!cyhal_gpio_read(CYBSP_USER_BTN)) { break; }
        }
    }

    /* User button is pressed. Format the block device. */
    printf("Formatting the block device...\n");
    lfs_format(&lfs, &lfs_cfg);
    printf("Formatting completed...\n");

#if(STORAGE_DEVICE_SD_CARD)
    lfs_sd_bd_destroy(&lfs_cfg);
#else
    lfs_spi_flash_bd_destroy(&lfs_cfg);
#endif /* #if(STORAGE_DEVICE_SD_CARD) */

    printf("Press reset to continue...\n");
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It does...
*    1. Initializes the UART for redirecting printf output
*    2. Intializes the user button GPIO
*    3. Creates a FreeRTOS task to perform file I/O operations
*    4. Starts the FreeRTOS scheduler
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* This enables RTOS aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Initialize the user button used for erasing the block device. */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Configure & the user button interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, user_button_interrupt_handler, NULL);

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************* "
           "Littlefs File System on SD Card and QSPI NOR Flash "
           "************* \n\n");
    
    /* Create the user tasks. See the respective task definition for more
     * details of these tasks.
     */
    xTaskCreate(littlefs_task, "Littlefs Task", LITTLEFS_TASK_STACK_SIZE,
                NULL, (configMAX_PRIORITIES - 1), &littlefs_task_handle);

    /* Start the RTOS scheduler. This function should never return */
    vTaskStartScheduler();

    (void) result; /* To avoid compiler warning */

    for (;;)
    {
    }
}

/* [] END OF FILE */
