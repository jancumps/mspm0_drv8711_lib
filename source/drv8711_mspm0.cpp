/**
 * mspm0 port of DRV8711 library https://github.com/jancumps/drv8711_lib
*/

module;

/* Including all the sysconfig generated defines */
#include "ti_msp_dl_config.h"

import drv8711;
import drv8711_config;

export module drv8711_mspm0;
export namespace drv8711_mspm0 {

class drv8711_mspm0 : public drv8711::drv8711_driver {
public:    
    drv8711_mspm0(SPI_Regs * spi, uint32_t cs,
        GPIO_Regs* gpio, uint32_t n_sleep, uint32_t reset) : drv8711::drv8711_driver(),
        spi_(spi), cs_(cs), gpio_(gpio), n_sleep_(n_sleep), reset_(reset) {}
    
    virtual void enable(bool enable) override {
        put(n_sleep_, enable);
    }

private:
    // initialise all registers from the defaults
    // defined in module drv8711_config
    // developer can override values before calling
    virtual void init_registers() {
        write(drv8711::reg_ctrl);
        write(drv8711::reg_torque);
        write(drv8711::reg_off);
        write(drv8711::reg_blank);
        write(drv8711::reg_decay);
        write(drv8711::reg_stall);
        write(drv8711::reg_drive);
        write(drv8711::reg_status);
    }
    
    virtual void init_spi() override{
        // Enable SPI at 1 MHz
        // cs is initiated in its own method
    }

    // users may want to use different CS with same SPI for several ICs
    // and can do that by overriding this method
    virtual void init_cs() override {
        put(cs_, false);
    }

    virtual void init_gpio() override{
        // nsleep as output
        put(n_sleep_, false);
        // reset as output
        put(reset_, false);
    }

    // write to a register
    virtual void write(uint16_t reg) override {
        SPI_Write16_Blocking(reg);
    }

    // read from a register
    virtual uint16_t read(uint16_t reg) override {
        uint16_t r_buffer;
        uint16_t w_buffer;
        w_buffer = reg | 0b1000;
        w_buffer = w_buffer << 12;
        r_buffer = SPI_WriteRead16_Blocking(w_buffer);
        return r_buffer & 0b0000111111111111; // first 4 read bits are undefined
    }

protected:
    SPI_Regs *spi_;
    uint32_t cs_; // chip select pin needs to be managed manually for this driver, as it needs to stay high accross the entire SPI transaction, which is not supported by the MSPM0 SPI driver
    GPIO_Regs* gpio_;
    uint32_t  n_sleep_;
    uint32_t  reset_;

private:
    void put(uint32_t pin, bool set) {
        DL_GPIO_writePinsVal(gpio_, pin, set ? pin : 0U);
    }

    void SPI_Write16_Blocking(uint16_t data) {
        // 1. Wait until there is space available in the SPI TX FIFO
        while (DL_SPI_isTXFIFOFull(spi_));
    
        // 2. Load the 16-bit data packet into the transmission register
        put(cs_, true);
        // Pad the setup time (Delays 31.25 ns, which satisfies 5 ns minimum requirement)
//        delay_cycles(1);
        DL_SPI_transmitData16(spi_, data);
    
        // 3. Block until the shift register completes transmission and the bus is idle
        while (DL_SPI_isBusy(spi_));
        // Pad the setup time (Delays 31.25 ns, which satisfies 5 ns minimum requirement)
        put(cs_, false);
    }

    uint16_t SPI_WriteRead16_Blocking(uint16_t dataToWrite) {
        uint16_t receivedData = 0;
        // 1. Flush any leftover garbage data out of the RX FIFO to ensure a clean transaction
        put(cs_, true);
        // Pad the setup time (Delays 31.25 ns, which satisfies 5 ns minimum requirement)
 //       delay_cycles(1);
        while (!DL_SPI_isRXFIFOEmpty(spi_)) {
            DL_SPI_receiveData16(spi_); // Read and discard
        }

        // 2. Wait until there is space available in the TX FIFO
        while (DL_SPI_isTXFIFOFull(spi_));
    
        // 3. Write the 16-bit variable into the transmission register
        DL_SPI_transmitData16(spi_, dataToWrite);
    
        // 4. Block until the RX FIFO receives the 16-bit response back from the slave
        while (DL_SPI_isRXFIFOEmpty(spi_));
    
        // 5. Extract and return the 16-bit data packet out of the RX FIFO
        receivedData = DL_SPI_receiveData16(spi_);
        // Pad the setup time (Delays 31.25 ns, which satisfies 5 ns minimum requirement)
//        delay_cycles(1);
        put(cs_, false);
        return receivedData;
    }

};

} // namespace drv8711_mspm0