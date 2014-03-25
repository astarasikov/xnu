/*
 * Copyright 2014 Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   If you are going to use this software in any form that does not involve
 *   releasing the source to this project or improving it, let me know beforehand.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Platform Expert for OMAP5432 UEVM
 */

#include <sys/types.h>
#include <mach/vm_param.h>
#include <machine/machine_routines.h>
#include <pexpert/device_tree.h>
#include <pexpert/protos.h>
#include <pexpert/pexpert.h>
#include <kern/debug.h>
#include <kern/simple_lock.h>
#include <machine/machine_routines.h>
#include <vm/pmap.h>
#include <arm/pmap.h>
#include <kern/cpu_data.h>

/* OMAP54XX CPU DEFINITIONS */
#define OMAP54XX_L4_CORE_BASE	0x4A000000
#define OMAP54XX_L4_WKUP_BASE	0x4AE00000
#define OMAP54XX_L4_PER_BASE	0x48000000
 
#define OMAP5_GIC_BASE 0x48211000
#define OMAP5_GIC_CPU_BASE 0x48212000

/* Debugging UART */
#define OMAP54XX_UART3_BASE		(OMAP54XX_L4_PER_BASE + 0x20000)

/* General Purpose Timer */
#define GPT2_BASE		(OMAP54XX_L4_PER_BASE  + 0x32000)

//#define OMAP5_TIMER_BASE GPT2_BASE
//#define OMAP5_TIMER_IRQ 38
//
#define OMAP5_TIMER_BASE 0x4ae18000
#define OMAP5_ARM_TIMER_BASE 0x4ae18000

/* BOARD CODE */
#define OMAP5_TIMER_BASE		GPT2_BASE
#define OMAP5_UART_BASE OMAP54XX_UART3_BASE
#define OMAP5_UART_BAUDRATE 115200
#define OMAP5_UART_CLOCK 48000000 

#define OMAP5_CLKCTRL_BASE 0x4ae07000
#define OMAP5_GPTIMER1_CLKCTRL 0x4ae07840

#ifdef BOARD_CONFIG_OMAP5432_UEVM

void Omap5_timer_enabled(int enable);
uint64_t Omap5_timer_value(void);
uint64_t Omap5_get_timebase(void);

#define mmio_read(a)    (*(volatile uint32_t *)(a))
#define mmio_write(a,v) (*(volatile uint32_t *)(a) = (v))
#define mmio_set(a,v)   mmio_write((a), mmio_read((a)) | (v))
#define mmio_clear(a,v) mmio_write((a), mmio_read((a)) & ~(v))

#define barrier()               __asm__ __volatile__("": : :"memory");

#define HwReg(x) *((volatile unsigned long*)(x))

#define KPRINTF_PREFIX  "PE_omap5432: "

extern void rtclock_intr(arm_saved_state_t * regs);
extern void rtc_configure(uint64_t hz);

vm_offset_t gOmapSerialUartBase = 0x0;
vm_offset_t gOmapInterruptControllerBase = 0x0;
vm_offset_t gOmapTimerBase = 0x0;
vm_offset_t gGICGICDistributorBase = 0x0;
vm_offset_t gGICCPUBase = 0x0;
vm_offset_t gOmapClkCtrlBase = 0x0;

static uint64_t clock_decrementer = 0;
static boolean_t clock_initialized = FALSE;
static boolean_t clock_had_irq = FALSE;
static uint64_t clock_absolute_time = 0;

#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_MASK(start, end)    (((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))

/******************************************************************************
 * OMAP5 TIMER
 *****************************************************************************/
#define V_SCLK	19200000
#define CONFIG_SYS_PTV 2

#define CONFIG_SYS_HZ 1000

#define TIMER_CLOCK		(V_SCLK / (2 << CONFIG_SYS_PTV))
#define TIMER_OVERFLOW_VAL	0xffffffff
#define TIMER_LOAD_VAL		0

enum {
	TIDR = 0,
	TIOCP_CFG = 0x10,
	TISR = 0x28,
	TIER = 0x2c,
	TICR = 0x30,
	TWER = 0x34,
	TCLR = 0x38,
	TCRR = 0x3c,
	TLDR = 0x40,
	TTGR = 0x44,
	TWPC = 0x48,
	TMAR = 0x4c,
	TCAR1 = 0x50,
	TCICR = 0x54,
	TCAR2 = 0x58,
};

#define TCLR_ST			(0x1 << 0)
#define TCLR_AR			(0x1 << 1)
#define TCLR_PRE		(0x1 << 5)

/* XXX
 * ARMv7 Timer 6144000
 *
 * */

static void timer_configure(void)
{
    gPEClockFrequencyInfo.timebase_frequency_hz = TIMER_CLOCK;

    clock_decrementer = CONFIG_SYS_HZ;
    kprintf(KPRINTF_PREFIX "decrementer frequency = %llu\n", clock_decrementer);

    rtc_configure(CONFIG_SYS_HZ);
    return;
}

uint64_t Omap5_timer_value(void)
{
	return HwReg(gOmapTimerBase + TCRR) / (TIMER_CLOCK / CONFIG_SYS_HZ);
}

void Omap5_timer_enabled(int enable)
{
    /*
     * Clear the TCLR [ST] bit 
     */
    if (enable)
        HwReg(gOmapTimerBase + TCLR) =
			(CONFIG_SYS_PTV << 2) | TCLR_PRE | TCLR_AR | TCLR_ST;
    else
        HwReg(gOmapTimerBase + TCLR) &= ~TCLR_ST;

#if 0
	if (enable) {
		HwReg(gARMTimerBase + 8) = ((1 << 5) | (1 << 7) | (1 << 6));
	}
	else {
		HwReg(gARMTimerBase + 8) &= ~(1 << 7);
	}
#endif

    return;
}

uint64_t Omap5_get_timebase(void)
{
    uint32_t timestamp;

    if (!clock_initialized)
        return 0;

    timestamp = Omap5_timer_value();

    if (timestamp) {
        uint64_t v = clock_absolute_time;
        v += (uint64_t) (((uint64_t) clock_decrementer) - (uint64_t) (timestamp));
        return v;
    } else {
        clock_absolute_time += clock_decrementer;
        return clock_absolute_time;
    }
}

void Omap5_timebase_init(void)
{
    /*
     * Stop the timer. 
     */
    Omap5_timer_enabled(FALSE);
    
	int i;
	for (i=0; i < 224; i += 32)
        HwReg(gGICGICDistributorBase + 0x100 + i * 4/32) = 0xffffffff;
	barrier();

    /*
     * Enable interrupts 
     */
    ml_set_interrupts_enabled(TRUE);

    /*
     * Set rtclock stuff 
     */
    timer_configure();

#if 0
    /*
     * Set timer decrementer defaults 
     */
    HwReg(gOmapTimerBase + TLDR) = 0xffffffe0;
    HwReg(gOmapTimerBase + TCRR) = 0xffffffe0;

    HwReg(gOmapTimerBase + TPIR) = 232000;
    HwReg(gOmapTimerBase + TNIR) = -768000;

    HwReg(gOmapTimerBase + TOCR) = 0;
    HwReg(gOmapTimerBase + TOWR) = 100;

    HwReg(gOmapTimerBase + TCLR) = (1 << 6);
#endif

    /*
     * !!! SET INTERRUPTS ENABLED ON OVERFLOW 
     */
    HwReg(gOmapTimerBase + TISR) = 0x7;
    HwReg(gOmapTimerBase + TIER) = 0x7;
	HwReg(gOmapTimerBase + 0x24) = 0;
	HwReg(gOmapTimerBase + TTGR) = 2;

	/* cm_wkup_gptimer1_clkctrl, GPTIMER1_CLKCTRL_CLKSEL_MASK set */	
	//HwReg(OMAP5_GPTIMER1_CLKCTRL) |= (1 << 24);

    /*
     * Arm the timer 
     */
    HwReg(gOmapTimerBase + TCLR) |= (1 << 0) | (1 << 1) | (2 << 10);

    /*
     * Wait for it. 
     */

	Omap5_timer_enabled(TRUE);
    clock_initialized = TRUE;

    //while (!clock_had_irq)
        barrier();

    kprintf(KPRINTF_PREFIX "timer is now up, ticks %llu\n", Omap5_timer_value());
    return;
}


/******************************************************************************
 * OMAP5 UART
 *****************************************************************************/

#define RBR     0x0
#define IER     0x4
#define FCR     0x8
#define LCR     0xC
#define MCR     0x10
#define LSR     0x14            // 335x
#define MSR     0x18
#define SCR     0x1C
#define SSR     0x44            // 335x

#define LCR_BKSE	0x80        /* Bank select enable */
#define LCR_8N1		0x03

#define SSR_TXFIFOFULL	0x01    /* 335x */

#define LSR_DR		0x01        /* Data ready */
#define LSR_THRE	0x20        /* Xmit holding register empty */

#define MCR_DTR         0x01
#define MCR_RTS         0x02

#define FCR_FIFO_EN     0x01    /* Fifo enable */
#define FCR_RXSR        0x02    /* Receiver soft reset */
#define FCR_TXSR        0x04    /* Transmitter soft reset */

#define LCRVAL LCR_8N1          /* 8 data, 1 stop, no parity */
#define MCRVAL (MCR_DTR | MCR_RTS)  /* RTS/DTR */
#define FCRVAL (FCR_FIFO_EN | FCR_RXSR | FCR_TXSR)  /* Clear & enable FIFOs */

#define THR     RBR
#define DLL     RBR
#define DLM     IER

void Omap5_early_putc(int c)
{
    if (c == '\n')
        Omap5_early_putc('\r');

    while (!(HwReg(OMAP5_UART_BASE + LSR) & LSR_THRE))
        barrier();

    HwReg(OMAP5_UART_BASE + THR) = c;
}

void Omap5_putc(int c)
{
    if (!gOmapSerialUartBase)
        return;

    if (c == '\n')
        Omap5_putc('\r');

    while (!(HwReg(gOmapSerialUartBase + LSR) & LSR_THRE))
        barrier();

    HwReg(gOmapSerialUartBase + THR) = c;
}

int Omap5_getc(void)
{
    int i = 0x20000;
    while (!(HwReg(gOmapSerialUartBase + LSR) & LSR_DR)) {
        i--; if(!i) return -1;
    }

    return (HwReg(gOmapSerialUartBase + RBR));
}

void Omap5_uart_init(void)
{
    int baudDivisor;
    gGICGICDistributorBase = ml_io_map(OMAP5_GIC_BASE, PAGE_SIZE);
    gGICCPUBase = ml_io_map(OMAP5_GIC_CPU_BASE, PAGE_SIZE);
    gOmapTimerBase = ml_io_map(OMAP5_TIMER_BASE, PAGE_SIZE);
	gOmapClkCtrlBase = ml_io_map(OMAP5_CLKCTRL_BASE, PAGE_SIZE);
    gOmapSerialUartBase = ml_io_map(OMAP5_UART_BASE, PAGE_SIZE);

    assert(OMAP5_UART_BAUDRATE != 0);
    baudDivisor = (OMAP5_UART_CLOCK / 16 / OMAP5_UART_BAUDRATE);

    HwReg(gOmapSerialUartBase + IER) = 0x00;
    HwReg(gOmapSerialUartBase + LCR) = LCR_BKSE | LCRVAL;
    HwReg(gOmapSerialUartBase + DLL) = baudDivisor & 0xFF;
    HwReg(gOmapSerialUartBase + DLM) = (baudDivisor >> 8) & 0xFF;
    HwReg(gOmapSerialUartBase + LCR) = LCRVAL;
    HwReg(gOmapSerialUartBase + MCR) = MCRVAL;
    HwReg(gOmapSerialUartBase + FCR) = FCRVAL;
}

/******************************************************************************
 * A15 GIC
 *****************************************************************************/
#define GIC_CPU_REG(off)            ((off))
#define GIC_DIST_REG(off)           ((off))

#define GIC_CPU_CTRL                GIC_CPU_REG(0x00)
#define GIC_CPU_PRIMASK             GIC_CPU_REG(0x04)
#define GIC_CPU_BINPOINT            GIC_CPU_REG(0x08)
#define GIC_CPU_INTACK              GIC_CPU_REG(0x0c)
#define GIC_CPU_EOI                 GIC_CPU_REG(0x10)
#define GIC_CPU_RUNNINGPRI          GIC_CPU_REG(0x14)
#define GIC_CPU_HIGHPRI             GIC_CPU_REG(0x18)

#define GIC_DIST_CTRL               GIC_DIST_REG(0x000)
#define GIC_DIST_CTR                GIC_DIST_REG(0x004)
#define GIC_DIST_ENABLE_SET         GIC_DIST_REG(0x100)
#define GIC_DIST_ENABLE_CLEAR       GIC_DIST_REG(0x180)
#define GIC_DIST_PENDING_SET        GIC_DIST_REG(0x200)
#define GIC_DIST_PENDING_CLEAR      GIC_DIST_REG(0x280)
#define GIC_DIST_ACTIVE_BIT         GIC_DIST_REG(0x300)
#define GIC_DIST_PRI                GIC_DIST_REG(0x400)
#define GIC_DIST_TARGET             GIC_DIST_REG(0x800)
#define GIC_DIST_CONFIG             GIC_DIST_REG(0xc00)
#define GIC_DIST_SOFTINT            GIC_DIST_REG(0xf00)

#define GIC_PPI_START 16
#define GIC_SPI_START 32


/* Intialize distributor */
static void gic_dist_init(void)
{
    uint32_t i;
    uint32_t num_irq = 0;
    uint32_t cpumask = 1;

    cpumask |= cpumask << 8;
    cpumask |= cpumask << 16;

    /* Disabling GIC */
    HwReg(gGICGICDistributorBase + GIC_DIST_CTRL) = 0;

    /*
     * Find out how many interrupts are supported.
     */
    num_irq = HwReg(gGICGICDistributorBase + GIC_DIST_CTR) & 0x1f;
    num_irq = (num_irq + 1) * 32;

    /* Set each interrupt line to use N-N software model
     * and edge sensitive, active high
     */
    for (i=32; i < num_irq; i += 16)
        HwReg(gGICGICDistributorBase + GIC_DIST_CONFIG + i * 4/16) = 0xffffffff;

    HwReg(gGICGICDistributorBase + GIC_DIST_CONFIG + 4) = 0xffffffff;

    /* Set up interrupts for this CPU */
    for (i = 32; i < num_irq; i += 4)
        HwReg(gGICGICDistributorBase + GIC_DIST_TARGET + i * 4 / 4) = cpumask;

    /* Set priority of all interrupts*/

    /*
     * In bootloader we dont care about priority so
     * setting up equal priorities for all
     */
    for (i=0; i < num_irq; i += 4)
        HwReg(gGICGICDistributorBase + GIC_DIST_PRI) = 0xa0a0a0a0;

    /* Disabling interrupts*/
    for (i=0; i < num_irq; i += 32)
        HwReg(gGICGICDistributorBase + GIC_DIST_ENABLE_CLEAR + i * 4/32) = 0xffffffff;

    HwReg(gGICGICDistributorBase + GIC_DIST_ENABLE_SET) = 0xffff;

    /*Enabling GIC*/
    HwReg(gGICGICDistributorBase + GIC_DIST_CTRL) = 0x1;
}

/* Intialize cpu specific controller */
static void gic_cpu_init(void)
{
    HwReg(gGICCPUBase + GIC_CPU_PRIMASK) = 0xf0;
    HwReg(gGICCPUBase + GIC_CPU_CTRL) = 0x1;
}

void GIC_interrupt_init(void)
{
    assert(gGICGICDistributorBase && gGICCPUBase);

    /* Initialize GIC. */
    gic_dist_init();
    gic_cpu_init();

    return;
}

void GIC_handle_interrupt(void *context)
{
    uint32_t irq_no = HwReg(gGICCPUBase + GIC_CPU_INTACK);
    if(irq_no > 224) {
        kprintf(KPRINTF_PREFIX "Got a bogus IRQ?");
        return;
    }
	kprintf(KPRINTF_PREFIX "IRQ %d\n", irq_no);

	if (irq_no == (32 + 38) || irq_no == (32 + 37)) {
		clock_had_irq = TRUE;
	}
	else {
        //irq_iokit_dispatch(irq_no);
    }

    /* EOI. */
    HwReg(gGICCPUBase + GIC_CPU_EOI) = irq_no;
    return;
}

/******************************************************************************
 * OMAP5 FrameBuffer
 *****************************************************************************/

/*
 * Stub for printing out to framebuffer.
 */
void vcputc(__unused int l, __unused int u, int c);

static void _fb_putc(int c)
{
    if (c == '\n') {
        vcputc(0, 0, '\r');
    }
    vcputc(0, 0, c);
    Omap5_putc(c);
}

void Omap5_framebuffer_init(void)
{
    //gOmapDisplayControllerBase = ml_io_map(OMAP5_DSS_BASE - 0x40, PAGE_SIZE);
#if 0
    /*
     * This *must* be page aligned. 
     */
    struct dispc_regs *OmapDispc = (struct dispc_regs *) (gOmapDisplayControllerBase + 0x440);

    uint32_t vs = OmapDispc->size_lcd;
    uint32_t lcd_width, lcd_height;

    lcd_height = (vs >> 16) + 1;
    lcd_width = (vs & 0xffff) + 1;
#else
	uint32_t lcd_width = 1280, lcd_height = 768;
    kprintf(KPRINTF_PREFIX "lcd size is %u x %u\n", lcd_width, lcd_height);

    /*
     * Allocate framebuffer 
     */
    void *framebuffer = pmap_steal_memory(lcd_width * lcd_width * 4);
    void *framebuffer_phys = pmap_extract(kernel_pmap, framebuffer);
    bzero(framebuffer, lcd_width * lcd_height * 4);
    kprintf(KPRINTF_PREFIX "software framebuffer at %p\n", framebuffer);

    /*
     * Hook to our framebuffer, the memory region should just be burned away from pmap...
     * (it isn't right now.)
     *
     * Or u-boot could just be fixed. 
     *
     * I wish we had a proper boot loader that did respect framebuffer memory regions...
     */
    PE_state.video.v_baseAddr = (unsigned long) 0x8f9c0000;
    PE_state.video.v_rowBytes = lcd_width * 2;
    PE_state.video.v_width = lcd_width;
    PE_state.video.v_height = lcd_height;
    PE_state.video.v_depth = 2 * (8);   // 32bpp

    kprintf(KPRINTF_PREFIX "framebuffer initialized\n");

    /*
     * Enable early framebuffer.
     */
    char tempbuf[16];

    if (PE_parse_boot_argn("-early-fb-debug", tempbuf, sizeof(tempbuf))) {
        initialize_screen((void *) &PE_state.video, kPEAcquireScreen);
    }

    if (PE_parse_boot_argn("-graphics-mode", tempbuf, sizeof(tempbuf))) {
        /*
         * BootX like framebuffer.
         */
        initialize_screen((void *) &PE_state.video, kPEGraphicsMode);
    } else {
        initialize_screen((void *) &PE_state.video, kPETextMode);
    }
#endif
    return;
}

/******************************************************************************
 * OMAP5 Board Init
 *****************************************************************************/

void Omap5_InitCaches(void)
{
    kprintf(KPRINTF_PREFIX "initializing i+dcache\n");
    cache_initialize();
    kprintf(KPRINTF_PREFIX "done\n");
}

void PE_init_SocSupport_omap5(void)
{
    gPESocDispatch.uart_getc = Omap5_getc;
    gPESocDispatch.uart_putc = Omap5_putc;
    gPESocDispatch.uart_init = Omap5_uart_init;

    gPESocDispatch.interrupt_init = GIC_interrupt_init;
    gPESocDispatch.timebase_init = Omap5_timebase_init;

    gPESocDispatch.get_timebase = Omap5_get_timebase;

    gPESocDispatch.handle_interrupt = GIC_handle_interrupt;

    gPESocDispatch.timer_value = Omap5_timer_value;
    gPESocDispatch.timer_enabled = Omap5_timer_enabled;

    gPESocDispatch.framebuffer_init = Omap5_framebuffer_init;

    Omap5_uart_init();
    PE_kputc = gPESocDispatch.uart_putc;

	kprintf(KPRINTF_PREFIX "PE_init_SocSupport_omap5 done\n");

    Omap5_framebuffer_init();
}

void PE_init_SocSupport_stub(void)
{
    PE_early_puts("PE_init_SocSupport: Initializing for OMAP5432\n");
    PE_init_SocSupport_omap5();
}

#endif

