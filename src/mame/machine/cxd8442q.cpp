// license:BSD-3-Clause
// copyright-holders:Brice Onken

/*
 * Sony CXD8442Q WSC-FIFOQ APbus FIFO and DMA controller
 *
 * The FIFOQ is an APbus DMA controller designed for interfacing some of the simpler and lower speed peripherals
 * to the APbus while providing DMA capabilities. Each FIFO chip can support up to 4 devices. There is no
 * documentation avaliable for these chips (that I have been able to find, anyways), so this implements the bare minimum
 * needed to satisfy the monitor ROM (for booting off of floppy drives) and NEWS-OS (for async serial communication).
 * I'm sure there is a lot of missing or hardware inaccurate functionality here - this was all derived from running stuff 
 * and observing the debugger/ emulator log files. Additionally, the way this is coded makes it interrupt pretty much whenever
 * data is avaliable. The real hardware probably buffers this more.
 *
 * The NWS-5000X uses two of these chips to drive the following peripherals:
 *  - Floppy disk controller
 *  - Sound
 *  - Asynchronous serial communication
 *  and potentially more.
 *
 * TODO:
 *  - Cleanup
 *  - Hardware-accurate behavior of the FIFO - this is a best guess.
 *  - Actual clock rate
 */

#include "cxd8442q.h"

#define VERBOSE 0
#include "logmacro.h"

DEFINE_DEVICE_TYPE(CXD8442Q, cxd8442q_device, "cxd8442q", "Sony CXD8442Q WSC-FIFOQ")

cxd8442q_device::cxd8442q_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : device_t(mconfig, CXD8442Q, tag, owner, clock), out_irq(*this),
                                                                                                                    fifo_channels{apfifo_channel(*this), apfifo_channel(*this), apfifo_channel(*this), apfifo_channel(*this)}
{
}

void cxd8442q_device::map(address_map &map)
{
    map.unmap_value_low();
    // Each channel has the same structure
    // The devices are mapped at the platform level, so this device only needs to handle the DMA and assorted details
    for (int channel = 0; channel < FIFO_CH_TOTAL; ++channel)
    {
        int channel_base = FIFO_REGION_OFFSET * channel; // start of a channel's registers

        map(channel_base + 0x00, channel_base + 0x03).lrw32(NAME(([this, channel]()
                                                                  { return fifo_channels[channel].mask; })),
                                                            NAME(([this, channel](uint32_t data)
                                                                  {
                                                                      LOG("FIFO CH%d: Setting mask to 0x%x\n", channel, data);
                                                                      fifo_channels[channel].mask = data;
                                                                  })));

        map(channel_base + 0x04, channel_base + 0x07).lrw32(NAME(([this, channel]()
                                                                  { return fifo_channels[channel].address; })),
                                                            NAME(([this, channel](uint32_t data)
                                                                  {
                                                                      LOG("FIFO CH%d: Setting address to 0x%x\n", channel, data);
                                                                      fifo_channels[channel].address = data;
                                                                  })));

        map(channel_base + 0x0c, channel_base + 0x0f).lrw32(NAME(([this, channel]()
                                                                  { return fifo_channels[channel].dma_mode; })),
                                                            NAME(([this, channel](uint32_t data)
                                                                  {
                                                                      LOG("FIFO CH%d: Setting DMA mode to 0x%x (%s)\n", channel, data, machine().describe_context());
                                                                      fifo_channels[channel].dma_mode = data;
                                                                      if (data & apfifo_channel::DMA_EN)
                                                                      {
                                                                          fifo_channels[channel].reset_for_transaction();
                                                                          fifo_timer->adjust(attotime::zero, 0, attotime::from_usec(DMA_TIMER));
                                                                      }
                                                                      else
                                                                      {
                                                                          fifo_channels[channel].intstat = 0x0; // XXX
                                                                      }
                                                                  })));

        map(channel_base + 0x18, channel_base + 0x1b).lrw32(NAME(([this, channel]()
                                                                  { return fifo_channels[channel].intctrl; })),
                                                            NAME(([this, channel](offs_t offset, uint32_t data)
                                                                  {
                                                                      LOG("FIFO CH%d: Set intctrl = 0x%x (%s)\n", channel, data, machine().describe_context());
                                                                      fifo_channels[channel].intctrl = data;
                                                                      irq_check();
                                                                  })));

        map(channel_base + 0x1c, channel_base + 0x1f).lr32(NAME(([this, channel]()
                                                                 { return fifo_channels[channel].intstat; })));

        map(channel_base + 0x30, channel_base + 0x33).lr32(NAME(([this, channel]()
                                                                 { return fifo_channels[channel].count; })));

        map(channel_base + 0x34, channel_base + 0x37).lrw8(NAME(([this, channel]()
                                                                 {
                                                                     if(fifo_channels[channel].count > 0)
                                                                     {
                                                                        fifo_channels[channel].count--;
                                                                        if(!fifo_channels[channel].count)
                                                                        {
                                                                            // time to clear interrupt if we got to 0
                                                                            fifo_channels[channel].intstat &= ~0x3;
                                                                        }
                                                                     }
                                                                     irq_check();
                                                                     return fifo_channels[channel].read_data_from_fifo();
                                                                 })),
                                                           NAME(([this, channel](offs_t offset, uint8_t data)
                                                                 {
                                                                     fifo_channels[channel].count++; // TODO: move to write_data_to_fifo
                                                                     LOG("FIFO CH%d: Pushing 0x%x, new count = 0x%x (%s)\n", channel, data, fifo_channels[channel].count, machine().describe_context());
                                                                     fifo_channels[channel].write_data_to_fifo(data);
                                                                 })));

    // These locations are written to a lot but not emulating them doesn't stop it from working for simple cases
    map(channel_base + 0x20, channel_base + 0x2f).noprw();
    }
}

void cxd8442q_device::map_fifo_ram(address_map &map)
{
    // TODO: experiment to see if this is readable/writeable from the real NWS5000X
    map(0x0, FIFO_MAX_RAM_SIZE - 1).lrw32(NAME(([this](offs_t offset)
                                                { return fifo_ram[offset]; })),
                                          NAME(([this](offs_t offset, uint32_t data)
                                                { fifo_ram[offset] = data; })));
}

void cxd8442q_device::device_add_mconfig(machine_config &config) {}

void cxd8442q_device::device_start()
{
    fifo_ram = std::make_unique<uint32_t[]>(FIFO_MAX_RAM_SIZE);
    save_pointer(NAME(fifo_ram), FIFO_MAX_RAM_SIZE);
    fifo_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(cxd8442q_device::fifo_dma_execute), this));
}

void cxd8442q_device::device_reset()
{
    for (int channel = 0; channel < FIFO_CH_TOTAL; ++channel)
    {
        fifo_channels[channel].reset();
    }
}

TIMER_CALLBACK_MEMBER(cxd8442q_device::fifo_dma_execute)
{
    bool dma_active = false;
    for (int channel = 0; channel < FIFO_CH_TOTAL; ++channel)
    {
        apfifo_channel &thisChannel = fifo_channels[channel];

        // Skip this channel if we don't need to do anything
        if (!(thisChannel.dma_mode & apfifo_channel::DMA_EN))
        {
            continue;
        }

        // Check DRQ to see if the device is ready to give or receive data
        if (thisChannel.drq_r())
        {
            // TODO: error condition?
            if(thisChannel.dma_cycle())
            {
                dma_active = true;
            }
        }
        else
        {
            dma_active = true;
        }
    }

    // If no channels were doing anything, we can terminate this activity for now
    if (!dma_active)
    {
        fifo_timer->adjust(attotime::never);
    }
}

bool apfifo_channel::dma_cycle()
{
    // TODO: Error handling (FIFO overrun, etc)
    bool stay_active = true;
    if (dma_r_callback != nullptr && (dma_mode & (DMA_DIRECTION | DMA_EN)) == DMA_EN)
    {
        // Grab our next chunk of data (might just be a byte, needs more investigation)
        fifo_device.fifo_ram[address + fifo_w_position] = dma_r_callback();
        ++count;

        // Increment and check if we need to wrap around
        if (++fifo_w_position > mask)
        {
            fifo_w_position = 0;
        }

        intstat = 3; // IRQ since we have data. This is likely not how the real chip works
        fifo_device.irq_check();
    }
    else if ((dma_w_callback != nullptr) && (count > 0) && ((dma_mode & (DMA_DIRECTION | DMA_EN)) == (DMA_DIRECTION | DMA_EN)))
    {
        // Move our next chunk of data from memory to the device
        dma_w_callback(fifo_device.fifo_ram[address + fifo_r_position]);
        --count;

        // Decrement and check if we need to wrap around
        if (++fifo_r_position > mask)
        {
            fifo_r_position = 0;
        }

        // Check if we are done
        if (count == 0)
        {
            stay_active = false;
            intstat = 3;
            fifo_device.irq_check();
        }
    }

    return stay_active;
}

void cxd8442q_device::irq_check()
{
    bool irqState = false;
    for (int channel = 0; channel < FIFO_CH_TOTAL; ++channel)
    {
        apfifo_channel &thisChannel = fifo_channels[channel];
        auto mask = thisChannel.intctrl & 0x1;
        if ((thisChannel.intstat & mask) != 0)
        {
            irqState = true;
        }
    }
    out_irq(irqState);
}

uint32_t apfifo_channel::read_data_from_fifo()
{
    // read data out of RAM at the current read position (relative to the start address)
    uint32_t value = fifo_device.fifo_ram[address + fifo_r_position];

    // Increment and check if we need to wrap around
    if (++fifo_r_position > mask)
    {
        fifo_r_position = 0;
    }

    // TODO: detect when FIFO has no valid data? Or is that a silent failure on the system?

    // Based on testing with the 5000X FDC subsystem, the monitor ROM uses `lb` commands in the 4-byte region
    // to read out the value of the 8-bit floppy data register. So, this ensures that the right data shows up
    // regardless of byte offset. Will need more experimentation to see if the FIFO always acts like that
    // or if the FDC is wired to cause this behavior.
    return (value << 24) | (value << 16) | (value << 8) | value;
}

void apfifo_channel::write_data_to_fifo(uint32_t data)
{
    fifo_device.fifo_ram[address + fifo_w_position] = data;

    // Increment and check if we need to wrap around
    if (++fifo_w_position > mask)
    {
        fifo_w_position = 0;
    }
}

void apfifo_channel::drq_w(int state)
{
    drq = state != 0;
    fifo_device.fifo_timer->adjust(attotime::zero, 0, attotime::from_usec(fifo_device.DMA_TIMER));
}