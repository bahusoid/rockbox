/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2006 Daniel Ankers
 * Copyright © 2008-2009 Rafaël Carré
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* Driver for the ARM PL180 SD/MMC controller inside AS3525 SoC */

#include "config.h" /* for HAVE_MULTIDRIVE & AMS_OF_SIZE */
#include "fs_defines.h"
#include "led.h"
#include "sdmmc.h"
#include "system.h"
#include "cpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gcc_extensions.h"
#include "as3525.h"
#include "pl180.h"  /* SD controller */
#include "pl081.h"  /* DMA controller */
#include "dma-target.h" /* DMA request lines */
#include "clock-target.h"
#include "panic.h"
#include "storage.h"

#ifdef HAVE_BUTTON_LIGHT
#include "backlight-target.h"
#endif

/*#define LOGF_ENABLE*/
#include "logf.h"

//#define VERIFY_WRITE 1

/* command flags */
#define MCI_NO_RESP     (0<<0)
#define MCI_RESP        (1<<0)
#define MCI_LONG_RESP   (1<<1)
#define MCI_ACMD        (1<<2)
#define MCI_NOCRC       (1<<3)

/* ARM PL180 registers */
#define MCI_POWER(i)       (*(volatile unsigned char *) (pl180_base[i]+0x00))
#define MCI_CLOCK(i)       (*(volatile unsigned long *) (pl180_base[i]+0x04))
#define MCI_ARGUMENT(i)    (*(volatile unsigned long *) (pl180_base[i]+0x08))
#define MCI_COMMAND(i)     (*(volatile unsigned long *) (pl180_base[i]+0x0C))
#define MCI_RESPCMD(i)     (*(volatile unsigned long *) (pl180_base[i]+0x10))
#define MCI_RESP0(i)       (*(volatile unsigned long *) (pl180_base[i]+0x14))
#define MCI_RESP1(i)       (*(volatile unsigned long *) (pl180_base[i]+0x18))
#define MCI_RESP2(i)       (*(volatile unsigned long *) (pl180_base[i]+0x1C))
#define MCI_RESP3(i)       (*(volatile unsigned long *) (pl180_base[i]+0x20))
#define MCI_DATA_TIMER(i)  (*(volatile unsigned long *) (pl180_base[i]+0x24))
#define MCI_DATA_LENGTH(i) (*(volatile unsigned short*) (pl180_base[i]+0x28))
#define MCI_DATA_CTRL(i)   (*(volatile unsigned char *) (pl180_base[i]+0x2C))
#define MCI_DATA_CNT(i)    (*(volatile unsigned short*) (pl180_base[i]+0x30))
#define MCI_STATUS(i)      (*(volatile unsigned long *) (pl180_base[i]+0x34))
#define MCI_CLEAR(i)       (*(volatile unsigned long *) (pl180_base[i]+0x38))
#define MCI_MASK0(i)       (*(volatile unsigned long *) (pl180_base[i]+0x3C))
#define MCI_MASK1(i)       (*(volatile unsigned long *) (pl180_base[i]+0x40))
#define MCI_SELECT(i)      (*(volatile unsigned long *) (pl180_base[i]+0x44))
#define MCI_FIFO_CNT(i)    (*(volatile unsigned long *) (pl180_base[i]+0x48))

#define MCI_DATA_ERROR    \
    ( MCI_DATA_CRC_FAIL   \
    | MCI_DATA_TIMEOUT    \
    | MCI_TX_UNDERRUN     \
    | MCI_RX_OVERRUN      \
    | MCI_START_BIT_ERR)

#define MCI_RESPONSE_ERROR    \
    ( MCI_CMD_TIMEOUT         \
    | MCI_CMD_CRC_FAIL)

#define MCI_FIFO(i)        ((unsigned long *) (pl180_base[i]+0x80))

#define IDE_INTERFACE_CLK  (1<<6) /* non AHB interface */
/* volumes */
#define     INTERNAL_AS3525 0   /* embedded SD card */
#define     SD_SLOT_AS3525   1   /* SD slot if present */

static const int pl180_base[NUM_DRIVES] = {
            NAND_FLASH_BASE
#ifdef HAVE_MULTIDRIVE
            , SD_MCI_BASE
#endif
};

static int sd_wait_for_tran_state(const int drive);
static int sd_select_bank(signed char bank);
static int sd_init_card(const int drive);
static void init_pl180_controller(const int drive);

#define BLOCKS_PER_BANK 0x7a7800u

static bool forceSlow = false;

static tCardInfo card_info[NUM_DRIVES];

/* maximum timeouts recommended in the SD Specification v2.00 */
/* MCI_DATA_TIMER register data timeout in card bus clock periods */
#define SD_MAX_READ_TIMEOUT     ((AS3525_PCLK_FREQ) / 1000 * 100) /* 100 ms */
#define SD_MAX_WRITE_TIMEOUT    ((AS3525_PCLK_FREQ) / 1000 * 250) /* 250 ms */

#ifdef CONFIG_STORAGE_MULTI
static int sd_first_drive = 0;
#else
#define    sd_first_drive 0
#endif

/* for compatibility */
static long last_disk_activity = -1;

static long stat_sd_reinint_count = 0;
static long stat_full_reinint_count = 0;
static long stat_sd_data_errors = 0;


#define MIN_YIELD_PERIOD 5  /* ticks */
static long next_yield = 0;

static struct mutex sd_mtx;

#if defined(HAVE_MULTIDRIVE)
static bool hs_card = false;
#define EXT_SD_BITS (1<<2)
#endif

#define PL180_MAX_TRANSFER_ERRORS 10

#define UNALIGNED_NUM_SECTORS 10
static unsigned char aligned_buffer[UNALIGNED_NUM_SECTORS* SD_BLOCK_SIZE] __attribute__((aligned(32)));   /* align on cache line size */
static unsigned char *uncached_buffer = AS3525_UNCACHED_ADDR(&aligned_buffer[0]);


static inline void mci_delay(void) { udelay(1000) ; }

static inline bool card_detect_target(void)
{
#if defined(HAVE_MULTIDRIVE)
    return !(GPIOA_PIN(2));
#else
    return false;
#endif
}

#if defined(HAVE_MULTIDRIVE) || defined(HAVE_HOTSWAP)
static void enable_controller_mci(bool on)
{

#if defined(HAVE_BUTTON_LIGHT) && defined(HAVE_MULTIDRIVE)
    extern int buttonlight_is_on;
#endif

#if defined(HAVE_HOTSWAP) && defined (HAVE_ADJUSTABLE_CPU_VOLTAGE)
    static bool cpu_boosted = false;
#endif
    static bool sd_enabled = true; /* force action on first call in sd_init() */

    if (sd_enabled == on)
        return; /* nothing to do */

    sd_enabled = on;

    if(on)
    {
#if defined(HAVE_BUTTON_LIGHT) && defined(HAVE_MULTIDRIVE)
        /* buttonlight AMSes need a bit of special handling for the buttonlight
         * here due to the dual mapping of GPIOD and XPD */
        bitmod32(&CCU_IO, 1<<2, 3<<2);  /* XPD is SD-MCI interface (b3:2 = 01) */
        if (buttonlight_is_on)
            GPIOD_DIR &= ~(1<<7);
        else
           buttonlight_hw_off();
#endif

#if defined(HAVE_HOTSWAP) && defined (HAVE_ADJUSTABLE_CPU_VOLTAGE)
        if(card_detect_target())  /* If SD card present Boost cpu for voltage */
        {
            cpu_boosted = true;
            cpu_boost(true);
        }
#endif  /* defined(HAVE_HOTSWAP) && defined (HAVE_ADJUSTABLE_CPU_VOLTAGE) */
    }
    else
    {
#if defined(HAVE_HOTSWAP) && defined (HAVE_ADJUSTABLE_CPU_VOLTAGE)
        if(cpu_boosted)
        {
            cpu_boost(false);
            cpu_boosted = false;
        }
#endif  /* defined(HAVE_HOTSWAP) && defined (HAVE_ADJUSTABLE_CPU_VOLTAGE) */

#if defined(HAVE_BUTTON_LIGHT) && defined(HAVE_MULTIDRIVE)
        bitmod32(&CCU_IO, 0<<2, 3<<2);  /* XPD is general purpose IO (b3:2 = 00) */
        if (buttonlight_is_on)
           buttonlight_hw_on();
#endif
    }
}
#endif /* defined(HAVE_MULTIDRIVE) || defined(HAVE_HOTSWAP) */

/* AMS v1 have two different drive interfaces MCI_SD(XPD) and GGU_IDE */
static void enable_controller(bool on, const int drive)
{

    if (drive == INTERNAL_AS3525)
    {
#ifndef BOOTLOADER
        if (on)
        {
            bitset32(&CGU_PERI, CGU_NAF_CLOCK_ENABLE);
            CGU_IDE |= IDE_INTERFACE_CLK;    /* interface enable */
        }
        else
        {
            CGU_IDE &= ~(IDE_INTERFACE_CLK);  /* interface disable */
            bitclr32(&CGU_PERI, CGU_NAF_CLOCK_ENABLE);
        }
#else
    (void) on;
#endif /* ndef BOOTLOADER */
    }
#if defined(HAVE_MULTIDRIVE) || defined(HAVE_HOTSWAP)
    else
        enable_controller_mci(on);
#endif
}

#ifdef HAVE_HOTSWAP
static int sd1_oneshot_callback(struct timeout *tmo)
{
    /* This is called only if the state was stable for 300ms - check state
     * and post appropriate event. */
    queue_broadcast(card_detect_target() ? SYS_HOTSWAP_INSERTED :
                                           SYS_HOTSWAP_EXTRACTED,
                    sd_first_drive + SD_SLOT_AS3525);
    return 0;
    (void)tmo;
}

void sd_gpioa_isr(void)
{
    static struct timeout sd1_oneshot;

    if (GPIOA_MIS & EXT_SD_BITS)
    {
        timeout_register(&sd1_oneshot, sd1_oneshot_callback, (3*HZ/10), 0);
        GPIOA_IC = EXT_SD_BITS; /* acknowledge interrupt */
    }
}
#endif  /* HAVE_HOTSWAP */

static bool send_cmd(const int drive, const int cmd, const int arg,
                     const int flags, long *response)
{
    int status;

    unsigned cmd_retries = 6;
    while(cmd_retries--)
    {
        if ((flags & MCI_ACMD) && /* send SD_APP_CMD before each try */
            !send_cmd(drive, SD_APP_CMD, card_info[drive].rca, MCI_RESP, response))
            return false;

        /* Clear old status flags */
        MCI_CLEAR(drive) = MCI_RESPONSE_ERROR | MCI_CMD_RESP_END | MCI_CMD_SENT;

        /* Load command argument or clear if none */
        MCI_ARGUMENT(drive) = arg;

        /* Construct MCI_COMMAND & enable CPSM */
        MCI_COMMAND(drive) =
           /*b0:5*/  cmd
           /* b6 */| ((flags & (MCI_RESP|MCI_LONG_RESP)) ? MCI_COMMAND_RESPONSE : 0)
           /* b7 */| ((flags & MCI_LONG_RESP) ? MCI_COMMAND_LONG_RESPONSE : 0)
           /* b8   | MCI_COMMAND_INTERRUPT */
           /* b9   | MCI_COMMAND_PENDING */  /*Only used with stream data transfer*/
           /* b10*/| MCI_COMMAND_ENABLE;     /* Enables CPSM */

        /* Wait while cmd completes then disable CPSM */
        while(MCI_STATUS(drive) & (MCI_CMD_ACTIVE | MCI_TX_ACTIVE));
        status = MCI_STATUS(drive);
        MCI_COMMAND(drive) = 0;

        /*  Handle command responses */
        if(flags & MCI_RESP)                /* CMD expects response */
        {
            response[0] = MCI_RESP0(drive); /* Always prepare short response */

            if(status & MCI_RESPONSE_ERROR) {/* timeout or crc failure */
                if ((status & MCI_CMD_CRC_FAIL) &&
                    (flags & MCI_NOCRC))
                    break;
                logf("sd cmd error: drive %d cmd %d arg %08x sd_status %08x resp0 %08lx",
                     drive, cmd, arg, status, response[0]);
                continue;
            }

            if(status & MCI_CMD_RESP_END)   /* Response passed CRC check */
            {
                if(flags & MCI_LONG_RESP)
                {   /* response[0] has already been read */
                    response[1] = MCI_RESP1(drive);
                    response[2] = MCI_RESP2(drive);
                    response[3] = MCI_RESP3(drive);
                }
                return true;
            }
        }
        else if(status & MCI_CMD_SENT)  /* CMD sent, no response required */
            return true;
    }

    return false;
}

/* MCI_CLOCK = MCLK / 2x(ClkDiv[bits 7:0]+1) */
#define MCI_FULLSPEED     (MCI_CLOCK_ENABLE | MCI_CLOCK_BYPASS)     /* MCLK   */
#define MCI_HALFSPEED     (MCI_CLOCK_ENABLE)                        /* MCLK/2 */
#define MCI_QUARTERSPEED  (MCI_CLOCK_ENABLE | 1)                    /* MCLK/4 */
#define MCI_IDENTSPEED    (MCI_CLOCK_ENABLE | AS3525_SD_IDENT_DIV)  /* IDENT  */

static int sd_init_card(const int drive)
{
    unsigned long response;
    long init_timeout;
    bool sd_v2 = false;

    card_info[drive].rca = 0;

    /* MCLCK on and set to 400kHz ident frequency  */
    MCI_CLOCK(drive) = MCI_IDENTSPEED;

    /* 100 - 400kHz clock required for Identification Mode  */
    /*  Start of Card Identification Mode ************************************/

    /* CMD0 Go Idle  -- all card functions switch back to default */
    if(!send_cmd(drive, SD_GO_IDLE_STATE, 0, MCI_NO_RESP, NULL))
        return -1;
    mci_delay();

    /* CMD8 Check for v2 sd card.  Must be sent before using ACMD41
      Non v2 cards will not respond to this command*/
    if(send_cmd(drive, SD_SEND_IF_COND, 0x1AA, MCI_RESP, &response))
        if((response & 0xFFF) == 0x1AA)
            sd_v2 = true;

    /* timeout for initialization is 1sec, from SD Specification 2.00 */
    init_timeout = current_tick + HZ;

    do {
        /* this timeout is the only valid error for this loop*/
        if(TIME_AFTER(current_tick, init_timeout))
            return -2;

        /* ACMD41 For v2 cards set HCS bit[30] & send host voltage range to all */
        send_cmd(drive, SD_APP_OP_COND, (0x00FF8000 | (sd_v2 ? 1<<30 : 0)),
                        MCI_ACMD|MCI_NOCRC|MCI_RESP, &card_info[drive].ocr);

    } while(!(card_info[drive].ocr & (1<<31)));

    /* CMD2 send CID */
    if(!send_cmd(drive, SD_ALL_SEND_CID, 0, MCI_RESP|MCI_LONG_RESP,
                            card_info[drive].cid))
        return -3;

    /* CMD3 send RCA */
    if(!send_cmd(drive, SD_SEND_RELATIVE_ADDR, 0, MCI_RESP,
                &card_info[drive].rca))
        return -4;

    /*  End of Card Identification Mode   ************************************/

#ifdef HAVE_MULTIDRIVE        /*  The internal SDs are v1 */

    /* Try to switch V2 cards to HS timings, non HS seem to ignore this */
    if(sd_v2)
    {
        /*  CMD7 w/rca: Select card to put it in TRAN state */
        if(!send_cmd(drive, SD_SELECT_CARD, card_info[drive].rca, MCI_RESP, &response))
            return -5;

        if(sd_wait_for_tran_state(drive))
            return -6;
        /* CMD6 0xf indicates no influence, [3:0],0x1 - HS Access*/
        if(!send_cmd(drive, SD_SWITCH_FUNC, 0x80fffff1, MCI_NO_RESP, NULL))
            return -7;
        sleep(HZ/10);/* need to wait at least 8 clock periods */

        /*  go back to STBY state so we can read csd */
        /*  CMD7 w/rca=0:  Deselect card to put it in STBY state */
        if(!send_cmd(drive, SD_DESELECT_CARD, 0, MCI_NO_RESP, NULL))
            return -8;
        mci_delay();
    }
#endif /*  HAVE_MULTIDRIVE  */

    /* CMD9 send CSD */
    if(!send_cmd(drive, SD_SEND_CSD, card_info[drive].rca,
                 MCI_RESP|MCI_LONG_RESP, card_info[drive].csd))
        return -9;

    sd_parse_csd(&card_info[drive]);

#if defined(HAVE_MULTIDRIVE)
    hs_card =  !forceSlow && (card_info[drive].speed == 50000000);
#endif

    /* Boost MCICLK to operating speed */
    if(drive == INTERNAL_AS3525)
        MCI_CLOCK(drive) = MCI_HALFSPEED;  /* MCICLK = IDE_CLK/2 = 25 MHz  */
#if defined(HAVE_MULTIDRIVE)
    else
        /* MCICLK = PCLK/2 = 31MHz(HS) or PCLK/4 = 15.5 Mhz (STD)*/
        MCI_CLOCK(drive) = (hs_card ? MCI_HALFSPEED : MCI_QUARTERSPEED);
#endif

    /*  CMD7 w/rca: Select card to put it in TRAN state */
    if(!send_cmd(drive, SD_SELECT_CARD, card_info[drive].rca, MCI_RESP, &response))
        return -10;

#if 0 /* FIXME : it seems that reading fails on some models */
    if(sd_wait_for_tran_state(drive) < 0)
        return -13;

    /* ACMD6: set bus width to 4-bit */
    if(!send_cmd(drive, SD_SET_BUS_WIDTH, 2, MCI_ACMD|MCI_RESP, &response))
        return -15;
    /* ACMD42: disconnect the pull-up resistor on CD/DAT3 */
    if(!send_cmd(drive, SD_SET_CLR_CARD_DETECT, 0, MCI_ACMD|MCI_RESP, &response))
        return -17;
    /* Now that card is widebus make controller aware */
    MCI_CLOCK(drive) |= MCI_CLOCK_WIDEBUS;
    mci_delay();

//    /*  CMD7 w/rca: Select card to put it in TRAN state */
//    if(!send_cmd(drive, SD_SELECT_CARD, card_info[drive].rca, MCI_RESP, &response))
//        return -19;
#endif


    /*
     * enable bank switching
     * without issuing this command, we only have access to 1/4 of the blocks
     * of the first bank (0x1E9E00 blocks, which is the size reported in the
     * CSD register)
     */
    if(drive == INTERNAL_AS3525)
    {
        const int ret = sd_select_bank(-1);
        if(ret < 0)
            return ret -16;

        /*  CMD7 w/rca = 0: Unselect card to put it in STBY state */
        if(!send_cmd(drive, SD_SELECT_CARD, 0, MCI_NO_RESP, NULL))
            return -17;
        mci_delay();

        /* CMD9 send CSD again, so we got the correct number of blocks */
        if(!send_cmd(drive, SD_SEND_CSD, card_info[drive].rca,
                     MCI_RESP|MCI_LONG_RESP, card_info[drive].csd))
            return -18;

        sd_parse_csd(&card_info[drive]);
        /* The OF is stored in the first blocks */
        card_info[INTERNAL_AS3525].numblocks -= AMS_OF_SIZE;

        /*  CMD7 w/rca: Select card to put it in TRAN state */
        if(!send_cmd(drive, SD_SELECT_CARD, card_info[drive].rca, MCI_RESP, &response))
            return -19;
    }

    if(sd_wait_for_tran_state(drive) < 0)
        return -13;

    card_info[drive].initialized = 1;

    return 0;
}

static void init_pl180_controller(const int drive)
{
    MCI_COMMAND(drive) = MCI_DATA_CTRL(drive) = 0;
    MCI_CLEAR(drive) = 0x7ff;

    MCI_MASK0(drive) = 0;
    MCI_MASK1(drive) = 0;
#ifdef HAVE_MULTIDRIVE
    VIC_INT_ENABLE = 0;
        //(drive == INTERNAL_AS3525) ? INTERRUPT_NAND : INTERRUPT_MCI0;
    /* clear previous irq */
    GPIOA_IC = EXT_SD_BITS;
    /* enable edge detecting */
    GPIOA_IS &= ~EXT_SD_BITS;
    /* detect both raising and falling edges */
    GPIOA_IBE |= EXT_SD_BITS;
    /* enable the card detect interrupt */
    GPIOA_IE |= EXT_SD_BITS;

#else
    VIC_INT_ENABLE = 0;
#endif

    MCI_POWER(drive) = MCI_POWER_UP | (MCI_VDD_3_5);  /*  OF Setting  */
    mci_delay();

    MCI_POWER(drive) |= MCI_POWER_ON;
    mci_delay();

    MCI_SELECT(drive) = 0;

    /* Pl180 clocks get turned on at start of card init */
}

int sd_init(void)
{
    int ret;
    CGU_IDE =   IDE_INTERFACE_CLK       /* enable interface */
            |   (AS3525_IDE_DIV << 2)
            |    AS3525_CLK_PLLA;       /* clock source = PLLA */

    bitset32(&CGU_PERI, CGU_NAF_CLOCK_ENABLE);
#ifdef HAVE_MULTIDRIVE
    bitset32(&CGU_PERI, CGU_MCI_CLOCK_ENABLE);
    bitmod32(&CCU_IO, 1<<2, 3<<2);  /* bits 3:2 = 01, xpd is SD interface */
#endif

    init_pl180_controller(INTERNAL_AS3525);
    ret = sd_init_card(INTERNAL_AS3525);
    if(ret < 0)
        return ret;
#ifdef HAVE_MULTIDRIVE
    init_pl180_controller(SD_SLOT_AS3525);
#endif

    /* init mutex */
    mutex_init(&sd_mtx);

    for (int i = 0; i < NUM_DRIVES ; i++)
        enable_controller(false, i);

    return 0;
}

#ifdef HAVE_HOTSWAP
bool sd_removable(IF_MD_NONVOID(int drive))
{
    return (drive == SD_SLOT_AS3525);
}

bool sd_present(IF_MD_NONVOID(int drive))
{
    return (drive == INTERNAL_AS3525) ? true : card_detect_target();
}
#endif /* HAVE_HOTSWAP */

volatile long lastResponse;

static int get_state(const int drive)
{
    unsigned long response = 0;
    
    if(!send_cmd(drive, SD_SEND_STATUS, card_info[drive].rca, MCI_RESP,
            &response))
        panicf( "mci status: %lu",  MCI_STATUS(drive));

    unsigned long x = ((response >> 9) & 0xf);
    return x;
}

static int sd_wait_for_tran_state(const int drive)
{
    unsigned long response = 0;
    unsigned int timeout = current_tick + 5 * HZ;

    while (1)
    {
        if(!send_cmd(drive, SD_SEND_STATUS, card_info[drive].rca, MCI_RESP,
                    &response))
            return -1;

        lastResponse = response;
        if (((response >> 9) & 0xf) == SD_TRAN)
            return 0;

        if(TIME_AFTER(current_tick, timeout))
            return -2;

        if (TIME_AFTER(current_tick, next_yield))
        {
            yield();
            next_yield = current_tick + MIN_YIELD_PERIOD;
        }
    }
}

static int sd_select_bank(signed char bank)
{
    int ret;
    unsigned loops = 0;

    memset(uncached_buffer, 0, 512);
    if(bank == -1)
    {   /* enable bank switching */
        uncached_buffer[0] = 16;
        uncached_buffer[1] = 1;
        uncached_buffer[2] = 10;
    }
    else
        uncached_buffer[0] = bank;

    int transfer_error = 0;
    do {
        if(loops++ > PL180_MAX_TRANSFER_ERRORS)
            panicf("SD bank %d error : 0x%x", bank,
                    transfer_error);

        ret = sd_wait_for_tran_state(INTERNAL_AS3525);
        if (ret < 0)
            return ret - 2;

        if(!send_cmd(INTERNAL_AS3525, SD_SWITCH_FUNC, 0x80ffffef, MCI_NO_RESP,
                     NULL))
            return -1;

        mci_delay();

        if(!send_cmd(INTERNAL_AS3525, 35, 0, MCI_NO_RESP, NULL))
            return -2;

        mci_delay();

        dma_retain();
        /* we don't use the uncached buffer here, because we need the
         * physical memory address for DMA transfers */
        dma_enable_channel(1, AS3525_PHYSICAL_ADDR(&aligned_buffer[0]),
            MCI_FIFO(INTERNAL_AS3525), DMA_PERI_SD,
            DMAC_FLOWCTRL_PERI_MEM_TO_PERI, true, false, 0, DMA_S8, NULL);

        MCI_DATA_TIMER(INTERNAL_AS3525) = SD_MAX_WRITE_TIMEOUT;
        MCI_DATA_LENGTH(INTERNAL_AS3525) = 512;
        MCI_DATA_CTRL(INTERNAL_AS3525) =  (1<<0) /* enable */   |
                                (0<<1) /* transfer direction */ |
                                (1<<3) /* DMA */                |
                                (9<<4) /* 2^9 = 512 */ ;

        /*  Wait for FIFO to empty, card may still be in PRG state  */
        while(! (MCI_STATUS(INTERNAL_AS3525) &  (MCI_DATA_ERROR | MCI_DATA_END)));
        while(MCI_STATUS(INTERNAL_AS3525) & MCI_TX_ACTIVE );
        transfer_error = MCI_STATUS(INTERNAL_AS3525) & MCI_DATA_ERROR;
        MCI_CLEAR(INTERNAL_AS3525) = MCI_DATA_ERROR | MCI_DATA_END;

        dma_release();

    } while(transfer_error);

    card_info[INTERNAL_AS3525].current_bank = (bank == -1) ? 0 : bank;

    return 0;
}

int totalWrites = 0;
static int sd_transfer_sectors(IF_MD(int drive,) unsigned long start,
                               int count, void* buf, const bool write)
{
    int lastTransfer = 0;

    int lastStatus = 0;
    int count_all = count;
    unsigned long response;
    int ret = 0;
#ifndef HAVE_MULTIDRIVE
    const int drive = 0;
#endif
    bool aligned = !((uintptr_t)buf & (CACHEALIGN_SIZE - 1));
    int retry_all = 10;
    int const retry_data_max = 2;
    int retry_data = retry_data_max;
    unsigned int real_numblocks;

    if(count < 1) /* XXX: why is it signed ? */
    {
        panicf("SD count:%d write:%d drive:%d", count, write, drive);
    }

    /* skip SanDisk OF */
    if (drive == INTERNAL_AS3525)
        start += AMS_OF_SIZE;

    enable_controller(true, drive);
    led(true);
    forceSlow = true;
//    if (forceSlow && !write)
//    {
//        forceSlow = false;
//        goto retry_with_reinit;
//    }

    while (card_info[drive].initialized<=0)
    {
        retry_with_reinit:
        if (--retry_all < 0)
            goto exit;
        ret = sd_init_card(drive);
    }

    /* Check the real block size after the card has been initialized */
    real_numblocks = card_info[drive].numblocks;
    /* 'start' represents the real (physical) starting sector
     *  so we must compare it to the real (physical) number of sectors */
    if (drive == INTERNAL_AS3525)
        real_numblocks += AMS_OF_SIZE;
    if ((start+count) > real_numblocks)
    {
        ret = -19;
        ++stat_full_reinint_count;
        goto retry_with_reinit;
    }

    if (sd_wait_for_tran_state(drive) != 0)
    {
//        led(false);
//        enable_controller(false, drive);
//
//        enable_controller(true, drive);
//        led(true);

        ret = -20;
        ++stat_full_reinint_count;
        goto retry_with_reinit;
    }

    dma_retain();

    if(aligned)
    {   /* direct transfer, indirect is always uncached */
        if(write)
            commit_dcache_range(buf, count * SECTOR_SIZE);
        else
            discard_dcache_range(buf, count * SECTOR_SIZE);
    }
    const int cmd = write ? SD_WRITE_MULTIPLE_BLOCK : SD_READ_MULTIPLE_BLOCK;
    while(count > 0)
    {
        /* 128 * 512 = 2^16, and doesn't fit in the 16 bits of DATA_LENGTH
         * register, so we have to transfer maximum 127 sectors at a time. */
        unsigned int transfer = (count >= 128) ? 127 : count; /* sectors */
        lastTransfer = transfer;
        void *dma_buf;

        unsigned long bank_start = start;

        /* Only switch banks for internal storage */
        if(drive == INTERNAL_AS3525)
        {
            unsigned int bank = 0;
            while(bank_start >= BLOCKS_PER_BANK)
            {
                bank_start -= BLOCKS_PER_BANK;
                bank++;
            }

            /* Switch bank if needed */
            if(card_info[INTERNAL_AS3525].current_bank != bank)
            {
                ret = sd_select_bank(bank);
                if (ret < 0)
                {
                    ret -= 20;
                    break;
                }
            }

            /* Do not cross a bank boundary in a single transfer loop */
            if((transfer + bank_start) > BLOCKS_PER_BANK)
                transfer = BLOCKS_PER_BANK - bank_start;
        }

        /* Set bank_start to the correct unit (blocks or bytes) */
        if(!(card_info[drive].ocr & (1<<30)))   /* not SDHC */
            bank_start *= SD_BLOCK_SIZE;

        if(!send_cmd(drive, cmd, bank_start, MCI_RESP, &response))
        {
            ret = -3*20;
            break;
        }

        if (response & SD_R1_CARD_ERROR)
        {
            ret = -4*20;
            break;
        }

        if(aligned)
        {
            dma_buf = AS3525_PHYSICAL_ADDR(buf);
        }
        else
        {
            dma_buf = AS3525_PHYSICAL_ADDR(&aligned_buffer[0]);
            if(transfer > UNALIGNED_NUM_SECTORS)
                transfer = UNALIGNED_NUM_SECTORS;

            if(write)
                memcpy(uncached_buffer, buf, transfer * SD_BLOCK_SIZE);
        }

        if(write)
        {
            dma_enable_channel(1, dma_buf, MCI_FIFO(drive),
                (drive == INTERNAL_AS3525) ? DMA_PERI_SD : DMA_PERI_SD_SLOT,
                DMAC_FLOWCTRL_PERI_MEM_TO_PERI, true, false, 0, DMA_S8, NULL);

            /*Small delay for writes prevents data crc failures at lower freqs*/
#ifdef HAVE_MULTIDRIVE
            if((drive == SD_SLOT_AS3525) && !hs_card)
                udelay(4);
#endif
        }
        else
            dma_enable_channel(1, MCI_FIFO(drive), dma_buf,
                (drive == INTERNAL_AS3525) ? DMA_PERI_SD : DMA_PERI_SD_SLOT,
                DMAC_FLOWCTRL_PERI_PERI_TO_MEM, false, true, 0, DMA_S8, NULL);

        MCI_DATA_TIMER(drive) = write ?
                SD_MAX_WRITE_TIMEOUT : SD_MAX_READ_TIMEOUT;
        MCI_DATA_LENGTH(drive) = transfer * SD_BLOCK_SIZE;
        MCI_DATA_CTRL(drive) =  (1<<0) /* enable */                     |
                                (!write<<1) /* transfer direction */    |
                                (1<<3) /* DMA */                        |
                                (9<<4) /* 2^9 = 512 */ ;

        while(! ( MCI_STATUS(drive) & (MCI_DATA_ERROR | MCI_DATA_END)));
        while(MCI_STATUS(drive) & MCI_TX_ACTIVE );
        const int transfer_error = MCI_STATUS(drive) & MCI_DATA_ERROR;
        MCI_CLEAR(drive) = MCI_DATA_ERROR | MCI_DATA_END;

        last_disk_activity = current_tick;
        lastStatus = MCI_STATUS(drive);
        
        if (!send_cmd(drive, SD_STOP_TRANSMISSION, 0, MCI_RESP, &response))
        {
            ret = -5*20;
            break;
        }

        if (sd_wait_for_tran_state(drive) != 0)
        {
            udelay(4);

//            led(false);
//            enable_controller(false, drive);
//
//            enable_controller(true, drive);
//            led(true);
//            mci_delay();
            if (sd_wait_for_tran_state(drive) != 0 && sd_init_card(drive) < 0)
            {
                ret = -6 * 20;
                break;
            }

            if (sd_wait_for_tran_state(drive) != 0)
            {
                ret = -7 * 20;
                break;
            }
            ++stat_sd_reinint_count;
        }
        /*
* If the write aborted early due to a tx underrun, disable the
* dma channel here, otherwise there are still 4 words in the fifo
* and the retried write will get corrupted.
*/
        dma_disable_channel(1);
        
        if(!transfer_error)
        {
            if(!write && !aligned)
                memcpy(buf, uncached_buffer, transfer * SD_BLOCK_SIZE);
            buf += transfer * SD_BLOCK_SIZE;
            start += transfer;
            count -= transfer;
            retry_data = retry_data_max;  /* reset errors counter */
            retry_all = 4;
            if(write)
                totalWrites += transfer;
        }
        else
        {
            if (--retry_data >= 0)
            {
                ++stat_sd_data_errors;
                continue;
            }
            ret = -24;
            break;
        }
    }
    dma_release();
    if (ret != 0) /* if we have error */
    {
        forceSlow = write ? true: false;
//        led(false);
//        enable_controller(false, drive);
//
//        enable_controller(true, drive);
//        led(true);
//        mci_delay();
        ++stat_full_reinint_count;
        goto retry_with_reinit;
    }

    exit:

    led(false);
    enable_controller(false, drive);

    if (ret)    /* error */
    {
        card_info[drive].initialized = 0;
        
        panicf("%serror:%d response:%lu, stop: %d, full: %d, data: %d, retry_all: %d, retry_data: %d, status: %d, total writes: %d", 
                 drive == INTERNAL_AS3525 ? "INTSD ": "", ret, response, stat_sd_reinint_count, stat_full_reinint_count, stat_sd_data_errors, retry_all, retry_data, lastStatus, totalWrites);
    }
//    if (forceSlow)
//    {
//        forceSlow = false;
//        card_info[drive].initialized = 0;
//    }
    return ret;
}

int sd_read_sectors(IF_MD(int drive,) unsigned long start, int count,
                     void* buf)
{
    int ret;

    mutex_lock(&sd_mtx);
    ret = sd_transfer_sectors(IF_MD(drive,) start, count, buf, false);
    mutex_unlock(&sd_mtx);

    return ret;
}

int sd_write_sectors(IF_MD(int drive,) unsigned long start, int count,
                     const void* buf)
{
#ifdef VERIFY_WRITE
    unsigned long saved_start = start;
    int saved_count = count;
    void *saved_buf = (void*)buf;
#endif
    int ret;

    mutex_lock(&sd_mtx);

    ret = sd_transfer_sectors(IF_MD(drive,) start, count, (void*)buf, true);

#ifdef VERIFY_WRITE
    if (ret) {
        /* write failed, no point in verifying */
        mutex_unlock(&sd_mtx);
        return ret;
    }

    count = saved_count;
    buf = saved_buf;
    start = saved_start;
    while (count) {
        int transfer = count;
        if(transfer > UNALIGNED_NUM_SECTORS)
            transfer = UNALIGNED_NUM_SECTORS;

        sd_transfer_sectors(IF_MD(drive,) start, transfer, aligned_buffer, false);
        if (memcmp(buf, aligned_buffer, transfer * 512) != 0) {
            /* try the write again in the hope to repair the damage */
            sd_transfer_sectors(IF_MD(drive,) saved_start, saved_count, saved_buf, true);
            panicf("sd: verify failed: sec=%ld n=%d!", start, transfer);
        }

        buf   += transfer * 512;
        count -= transfer;
        start += transfer;
    }
#endif

    mutex_unlock(&sd_mtx);

    return ret;
}

long sd_last_disk_activity(void)
{
    return last_disk_activity;
}

tCardInfo *card_get_info_target(int card_no)
{
    return &card_info[card_no];
}

#ifdef CONFIG_STORAGE_MULTI
int sd_num_drives(int first_drive)
{
    sd_first_drive = first_drive;
    return NUM_DRIVES;
}
#endif /* CONFIG_STORAGE_MULTI */

void ams_sd_get_debug_info(struct ams_sd_debug_info *info)
{
    #define MCI_NAND *((volatile unsigned long *)(NAND_FLASH_BASE + 0x04))
    #define MCI_SD   *((volatile unsigned long *)(SD_MCI_BASE + 0x04))

    mutex_lock(&sd_mtx);

    for (int i = 0; i < NUM_DRIVES ; i++)
        enable_controller(true, i); /* must be on to read regs */

    info->mci_nand = MCI_NAND;
#ifdef HAVE_MULTIDRIVE
    info->mci_sd = MCI_SD;
#endif

    info->sd_reinit_count = stat_sd_reinint_count;
    info->full_reinit_count = stat_full_reinint_count;
    info->data_error_count = stat_sd_data_errors;

    for (int i = 0; i < NUM_DRIVES ; i++)
        enable_controller(false, i);

    mutex_unlock(&sd_mtx);
}

int sd_event(long id, intptr_t data)
{
    int rc = 0;

    switch (id)
    {
#ifdef HAVE_HOTSWAP
    case SYS_HOTSWAP_INSERTED:
    case SYS_HOTSWAP_EXTRACTED:
        mutex_lock(&sd_mtx); /* lock-out card activity */

        /* Force card init for new card, re-init for re-inserted one or
         * clear if the last attempt to init failed with an error. */
        card_info[data].initialized = 0;

        if (id == SYS_HOTSWAP_INSERTED)
        {
            enable_controller(true, data);
            init_pl180_controller(data);
            rc = sd_init_card(data);
            enable_controller(false, data);
        }

        mutex_unlock(&sd_mtx);
        break;
#endif /* HAVE_HOTSWAP */
    case Q_STORAGE_TICK:
        /* never let a timer wrap confuse us */
        next_yield = current_tick;
    default:
        rc = storage_event_default_handler(id, data, last_disk_activity,
                                           STORAGE_SD);
        break;
    }

    return rc;
}
