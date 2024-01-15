/*

Copyright (c) 2015-2017 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

// Language: Verilog 2001

`timescale 1ns / 1ps

/*
 * I2C master
 */
//module i2c_master (
//    input  wire        clk,
//    input  wire        rst,
//
//    /*
//     * Host interface
//     */
//    input  wire [6:0]  s_axis_cmd_address,
//    input  wire        s_axis_cmd_start,
//    input  wire        s_axis_cmd_read,
//    input  wire        s_axis_cmd_write,
//    input  wire        s_axis_cmd_write_multiple,
//    input  wire        s_axis_cmd_stop,
//    input  wire        s_axis_cmd_valid,
//    output wire        s_axis_cmd_ready,
//
//    input  wire [7:0]  s_axis_data_tdata,
//    input  wire        s_axis_data_tvalid,
//    output wire        s_axis_data_tready,
//    input  wire        s_axis_data_tlast,
//
//    output wire [7:0]  m_axis_data_tdata,
//    output wire        m_axis_data_tvalid,
//    input  wire        m_axis_data_tready,
//    output wire        m_axis_data_tlast,
//
//    /*
//     * I2C interface
//     */
//    input  wire        scl_i,
//    output wire        scl_o,
//    output wire        scl_t,
//    input  wire        sda_i,
//    output wire        sda_o,
//    output wire        sda_t,
//
//    /*
//     * Status
//     */
//    output wire        busy,
//    output wire        bus_control,
//    output wire        bus_active,
//    output wire        missed_ack,
//
//    /*
//     * Configuration
//     */
//    input  wire [15:0] prescale,
//    input  wire        stop_on_idle
//);
//
///*
//
//I2C
//
//Read
//    __    ___ ___ ___ ___ ___ ___ ___         ___ ___ ___ ___ ___ ___ ___ ___     ___ ___ ___ ___ ___ ___ ___ ___        __
//sda   \__/_6_X_5_X_4_X_3_X_2_X_1_X_0_\_R___A_/_7_X_6_X_5_X_4_X_3_X_2_X_1_X_0_\_A_/_7_X_6_X_5_X_4_X_3_X_2_X_1_X_0_\_A____/
//    ____   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   ____
//scl  ST \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ SP
//
//Write
//    __    ___ ___ ___ ___ ___ ___ ___ ___     ___ ___ ___ ___ ___ ___ ___ ___     ___ ___ ___ ___ ___ ___ ___ ___ ___    __
//sda   \__/_6_X_5_X_4_X_3_X_2_X_1_X_0_/ W \_A_/_7_X_6_X_5_X_4_X_3_X_2_X_1_X_0_\_A_/_7_X_6_X_5_X_4_X_3_X_2_X_1_X_0_/ N \__/
//    ____   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   ____
//scl  ST \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ \_/ SP
//
//Commands:
//
//read
//    read data byte
//    set start to force generation of a start condition
//    start is implied when bus is inactive or active with write or different address
//    set stop to issue a stop condition after reading current byte
//    if stop is set with read command, then m_axis_data_tlast will be set
//
//write
//    write data byte
//    set start to force generation of a start condition
//    start is implied when bus is inactive or active with read or different address
//    set stop to issue a stop condition after writing current byte
//
//write multiple
//    write multiple data bytes (until s_axis_data_tlast)
//    set start to force generation of a start condition
//    start is implied when bus is inactive or active with read or different address
//    set stop to issue a stop condition after writing block
//
//stop
//    issue stop condition if bus is active
//
//Status:
//
//busy
//    module is communicating over the bus
//
//bus_control
//    module has control of bus in active state
//
//bus_active
//    bus is active, not necessarily controlled by this module
//
//missed_ack
//    strobed when a slave ack is missed
//
//Parameters:
//
//prescale
//    set prescale to 1/4 of the minimum clock period in units
//    of input clk cycles (prescale = Fclk / (FI2Cclk * 4))
//
//stop_on_idle
//    automatically issue stop when command input is not valid
//
//Example of interfacing with tristate pins:
//(this will work for any tristate bus)
//
//assign scl_i = scl_pin;
//assign scl_pin = scl_t ? 1'bz : scl_o;
//assign sda_i = sda_pin;
//assign sda_pin = sda_t ? 1'bz : sda_o;
//
//Equivalent code that does not use *_t connections:
//(we can get away with this because I2C is open-drain)
//
//assign scl_i = scl_pin;
//assign scl_pin = scl_o ? 1'bz : 1'b0;
//assign sda_i = sda_pin;
//assign sda_pin = sda_o ? 1'bz : 1'b0;
//
//Example of two interconnected I2C devices:
//
//assign scl_1_i = scl_1_o & scl_2_o;
//assign scl_2_i = scl_1_o & scl_2_o;
//assign sda_1_i = sda_1_o & sda_2_o;
//assign sda_2_i = sda_1_o & sda_2_o;
//
//Example of two I2C devices sharing the same pins:
//
//assign scl_1_i = scl_pin;
//assign scl_2_i = scl_pin;
//assign scl_pin = (scl_1_o & scl_2_o) ? 1'bz : 1'b0;
//assign sda_1_i = sda_pin;
//assign sda_2_i = sda_pin;
//assign sda_pin = (sda_1_o & sda_2_o) ? 1'bz : 1'b0;
//
//Notes:
//
//scl_o should not be connected directly to scl_i, only via AND logic or a tristate
//I/O pin.  This would prevent devices from stretching the clock period.
//
//*/
//
//localparam [4:0]
//    STATE_IDLE = 4'd0,
//    STATE_ACTIVE_WRITE = 4'd1,
//    STATE_ACTIVE_READ = 4'd2,
//    STATE_START_WAIT = 4'd3,
//    STATE_START = 4'd4,
//    STATE_ADDRESS_1 = 4'd5,
//    STATE_ADDRESS_2 = 4'd6,
//    STATE_WRITE_1 = 4'd7,
//    STATE_WRITE_2 = 4'd8,
//    STATE_WRITE_3 = 4'd9,
//    STATE_READ = 4'd10,
//    STATE_STOP = 4'd11;
//
//reg [4:0] state_reg = STATE_IDLE, state_next;
//
//localparam [4:0]
//    PHY_STATE_IDLE = 5'd0,
//    PHY_STATE_ACTIVE = 5'd1,
//    PHY_STATE_REPEATED_START_1 = 5'd2,
//    PHY_STATE_REPEATED_START_2 = 5'd3,
//    PHY_STATE_START_1 = 5'd4,
//    PHY_STATE_START_2 = 5'd5,
//    PHY_STATE_WRITE_BIT_1 = 5'd6,
//    PHY_STATE_WRITE_BIT_2 = 5'd7,
//    PHY_STATE_WRITE_BIT_3 = 5'd8,
//    PHY_STATE_READ_BIT_1 = 5'd9,
//    PHY_STATE_READ_BIT_2 = 5'd10,
//    PHY_STATE_READ_BIT_3 = 5'd11,
//    PHY_STATE_READ_BIT_4 = 5'd12,
//    PHY_STATE_STOP_1 = 5'd13,
//    PHY_STATE_STOP_2 = 5'd14,
//    PHY_STATE_STOP_3 = 5'd15;
//
//reg [4:0] phy_state_reg = STATE_IDLE, phy_state_next;
//
//reg phy_start_bit;
//reg phy_stop_bit;
//reg phy_write_bit;
//reg phy_read_bit;
//reg phy_release_bus;
//
//reg phy_tx_data;
//
//reg phy_rx_data_reg = 1'b0, phy_rx_data_next;
//
//reg [6:0] addr_reg = 7'd0, addr_next;
//reg [7:0] data_reg = 8'd0, data_next;
//reg last_reg = 1'b0, last_next;
//
//reg mode_read_reg = 1'b0, mode_read_next;
//reg mode_write_multiple_reg = 1'b0, mode_write_multiple_next;
//reg mode_stop_reg = 1'b0, mode_stop_next;
//
//reg [16:0] delay_reg = 16'd0, delay_next;
//reg delay_scl_reg = 1'b0, delay_scl_next;
//reg delay_sda_reg = 1'b0, delay_sda_next;
//
//reg [3:0] bit_count_reg = 4'd0, bit_count_next;
//
//reg s_axis_cmd_ready_reg = 1'b0, s_axis_cmd_ready_next;
//
//reg s_axis_data_tready_reg = 1'b0, s_axis_data_tready_next;
//
//reg [7:0] m_axis_data_tdata_reg = 8'd0, m_axis_data_tdata_next;
//reg m_axis_data_tvalid_reg = 1'b0, m_axis_data_tvalid_next;
//reg m_axis_data_tlast_reg = 1'b0, m_axis_data_tlast_next;
//
//reg scl_i_reg = 1'b1;
//reg sda_i_reg = 1'b1;
//
//reg scl_o_reg = 1'b1, scl_o_next;
//reg sda_o_reg = 1'b1, sda_o_next;
//
//reg last_scl_i_reg = 1'b1;
//reg last_sda_i_reg = 1'b1;
//
//reg busy_reg = 1'b0;
//reg bus_active_reg = 1'b0;
//reg bus_control_reg = 1'b0, bus_control_next;
//reg missed_ack_reg = 1'b0, missed_ack_next;
//
//assign s_axis_cmd_ready = s_axis_cmd_ready_reg;
//
//assign s_axis_data_tready = s_axis_data_tready_reg;
//
//assign m_axis_data_tdata = m_axis_data_tdata_reg;
//assign m_axis_data_tvalid = m_axis_data_tvalid_reg;
//assign m_axis_data_tlast = m_axis_data_tlast_reg;
//
//assign scl_o = scl_o_reg;
//assign scl_t = scl_o_reg;
//assign sda_o = sda_o_reg;
//assign sda_t = sda_o_reg;
//
//assign busy = busy_reg;
//assign bus_active = bus_active_reg;
//assign bus_control = bus_control_reg;
//assign missed_ack = missed_ack_reg;
//
//wire scl_posedge = scl_i_reg & ~last_scl_i_reg;
//wire scl_negedge = ~scl_i_reg & last_scl_i_reg;
//wire sda_posedge = sda_i_reg & ~last_sda_i_reg;
//wire sda_negedge = ~sda_i_reg & last_sda_i_reg;
//
//wire start_bit = sda_negedge & scl_i_reg;
//wire stop_bit = sda_posedge & scl_i_reg;
//
//always @* begin
//    state_next = STATE_IDLE;
//
//    phy_start_bit = 1'b0;
//    phy_stop_bit = 1'b0;
//    phy_write_bit = 1'b0;
//    phy_read_bit = 1'b0;
//    phy_tx_data = 1'b0;
//    phy_release_bus = 1'b0;
//
//    addr_next = addr_reg;
//    data_next = data_reg;
//    last_next = last_reg;
//
//    mode_read_next = mode_read_reg;
//    mode_write_multiple_next = mode_write_multiple_reg;
//    mode_stop_next = mode_stop_reg;
//
//    bit_count_next = bit_count_reg;
//
//    s_axis_cmd_ready_next = 1'b0;
//
//    s_axis_data_tready_next = 1'b0;
//
//    m_axis_data_tdata_next = m_axis_data_tdata_reg;
//    m_axis_data_tvalid_next = m_axis_data_tvalid_reg & ~m_axis_data_tready;
//    m_axis_data_tlast_next = m_axis_data_tlast_reg;
//
//    missed_ack_next = 1'b0;
//
//    // generate delays
//    if (phy_state_reg != PHY_STATE_IDLE && phy_state_reg != PHY_STATE_ACTIVE) begin
//        // wait for phy operation
//        state_next = state_reg;
//    end else begin
//        // process states
//        case (state_reg)
//            STATE_IDLE: begin
//                // line idle
//                s_axis_cmd_ready_next = 1'b1;
//
//                if (s_axis_cmd_ready & s_axis_cmd_valid) begin
//                    // command valid
//                    if (s_axis_cmd_read ^ (s_axis_cmd_write | s_axis_cmd_write_multiple)) begin
//                        // read or write command
//                        addr_next = s_axis_cmd_address;
//                        mode_read_next = s_axis_cmd_read;
//                        mode_write_multiple_next = s_axis_cmd_write_multiple;
//                        mode_stop_next = s_axis_cmd_stop;
//
//                        s_axis_cmd_ready_next = 1'b0;
//
//                        // start bit
//                        if (bus_active) begin
//                            state_next = STATE_START_WAIT;
//                        end else begin
//                            phy_start_bit = 1'b1;
//                            bit_count_next = 4'd8;
//                            state_next = STATE_ADDRESS_1;
//                        end
//                    end else begin
//                        // invalid or unspecified - ignore
//                        state_next = STATE_IDLE;
//                    end
//                end else begin
//                    state_next = STATE_IDLE;
//                end
//            end
//            STATE_ACTIVE_WRITE: begin
//                // line active with current address and read/write mode
//                s_axis_cmd_ready_next = 1'b1;
//
//                if (s_axis_cmd_ready & s_axis_cmd_valid) begin
//                    // command valid
//                    if (s_axis_cmd_read ^ (s_axis_cmd_write | s_axis_cmd_write_multiple)) begin
//                        // read or write command
//                        addr_next = s_axis_cmd_address;
//                        mode_read_next = s_axis_cmd_read;
//                        mode_write_multiple_next = s_axis_cmd_write_multiple;
//                        mode_stop_next = s_axis_cmd_stop;
//
//                        s_axis_cmd_ready_next = 1'b0;
//                        
//                        if (s_axis_cmd_start || s_axis_cmd_address != addr_reg || s_axis_cmd_read) begin
//                            // address or mode mismatch or forced start - repeated start
//
//                            // repeated start bit
//                            phy_start_bit = 1'b1;
//                            bit_count_next = 4'd8;
//                            state_next = STATE_ADDRESS_1;
//                        end else begin
//                            // address and mode match
//
//                            // start write
//                            s_axis_data_tready_next = 1'b1;
//                            state_next = STATE_WRITE_1;
//                        end
//                    end else if (s_axis_cmd_stop && !(s_axis_cmd_read || s_axis_cmd_write || s_axis_cmd_write_multiple)) begin
//                        // stop command
//                        phy_stop_bit = 1'b1;
//                        state_next = STATE_IDLE;
//                    end else begin
//                        // invalid or unspecified - ignore
//                        state_next = STATE_ACTIVE_WRITE;
//                    end
//                end else begin
//                    if (stop_on_idle & s_axis_cmd_ready & ~s_axis_cmd_valid) begin
//                        // no waiting command and stop_on_idle selected, issue stop condition
//                        phy_stop_bit = 1'b1;
//                        state_next = STATE_IDLE;
//                    end else begin
//                        state_next = STATE_ACTIVE_WRITE;
//                    end
//                end
//            end
//            STATE_ACTIVE_READ: begin
//                // line active to current address
//                s_axis_cmd_ready_next = ~m_axis_data_tvalid;
//
//                if (s_axis_cmd_ready & s_axis_cmd_valid) begin
//                    // command valid
//                    if (s_axis_cmd_read ^ (s_axis_cmd_write | s_axis_cmd_write_multiple)) begin
//                        // read or write command
//                        addr_next = s_axis_cmd_address;
//                        mode_read_next = s_axis_cmd_read;
//                        mode_write_multiple_next = s_axis_cmd_write_multiple;
//                        mode_stop_next = s_axis_cmd_stop;
//
//                        s_axis_cmd_ready_next = 1'b0;
//                        
//                        if (s_axis_cmd_start || s_axis_cmd_address != addr_reg || s_axis_cmd_write) begin
//                            // address or mode mismatch or forced start - repeated start
//
//                            // write nack for previous read
//                            phy_write_bit = 1'b1;
//                            phy_tx_data = 1'b1;
//                            // repeated start bit
//                            state_next = STATE_START;
//                        end else begin
//                            // address and mode match
//
//                            // write ack for previous read
//                            phy_write_bit = 1'b1;
//                            phy_tx_data = 1'b0;
//                            // start next read
//                            bit_count_next = 4'd8;
//                            data_next = 8'd0;
//                            state_next = STATE_READ;
//                        end
//                    end else if (s_axis_cmd_stop && !(s_axis_cmd_read || s_axis_cmd_write || s_axis_cmd_write_multiple)) begin
//                        // stop command
//                        // write nack for previous read
//                        phy_write_bit = 1'b1;
//                        phy_tx_data = 1'b1;
//                        // send stop bit
//                        state_next = STATE_STOP;
//                    end else begin
//                        // invalid or unspecified - ignore
//                        state_next = STATE_ACTIVE_READ;
//                    end
//                end else begin
//                    if (stop_on_idle & s_axis_cmd_ready & ~s_axis_cmd_valid) begin
//                        // no waiting command and stop_on_idle selected, issue stop condition
//                        // write ack for previous read
//                        phy_write_bit = 1'b1;
//                        phy_tx_data = 1'b1;
//                        // send stop bit
//                        state_next = STATE_STOP;
//                    end else begin
//                        state_next = STATE_ACTIVE_READ;
//                    end
//                end
//            end
//            STATE_START_WAIT: begin
//                // wait for bus idle
//
//                if (bus_active) begin
//                    state_next = STATE_START_WAIT;
//                end else begin
//                    // bus is idle, take control
//                    phy_start_bit = 1'b1;
//                    bit_count_next = 4'd8;
//                    state_next = STATE_ADDRESS_1;
//                end
//            end
//            STATE_START: begin
//                // send start bit
//
//                phy_start_bit = 1'b1;
//                bit_count_next = 4'd8;
//                state_next = STATE_ADDRESS_1;
//            end
//            STATE_ADDRESS_1: begin
//                // send address
//                bit_count_next = bit_count_reg - 1;
//                if (bit_count_reg > 1) begin
//                    // send address
//                    phy_write_bit = 1'b1;
//                    phy_tx_data = addr_reg[bit_count_reg-2];
//                    state_next = STATE_ADDRESS_1;
//                end else if (bit_count_reg > 0) begin
//                    // send read/write bit
//                    phy_write_bit = 1'b1;
//                    phy_tx_data = mode_read_reg;
//                    state_next = STATE_ADDRESS_1;
//                end else begin
//                    // read ack bit
//                    phy_read_bit = 1'b1;
//                    state_next = STATE_ADDRESS_2;
//                end
//            end
//            STATE_ADDRESS_2: begin
//                // read ack bit
//                missed_ack_next = phy_rx_data_reg;
//
//                if (mode_read_reg) begin
//                    // start read
//                    bit_count_next = 4'd8;
//                    data_next = 1'b0;
//                    state_next = STATE_READ;
//                end else begin
//                    // start write
//                    s_axis_data_tready_next = 1'b1;
//                    state_next = STATE_WRITE_1;
//                end
//            end
//            STATE_WRITE_1: begin
//                s_axis_data_tready_next = 1'b1;
//
//                if (s_axis_data_tready & s_axis_data_tvalid) begin
//                    // got data, start write
//                    data_next = s_axis_data_tdata;
//                    last_next = s_axis_data_tlast;
//                    bit_count_next = 4'd8;
//                    s_axis_data_tready_next = 1'b0;
//                    state_next = STATE_WRITE_2;
//                end else begin
//                    // wait for data
//                    state_next = STATE_WRITE_1;
//                end
//            end
//            STATE_WRITE_2: begin
//                // send data
//                bit_count_next = bit_count_reg - 1;
//                if (bit_count_reg > 0) begin
//                    // write data bit
//                    phy_write_bit = 1'b1;
//                    phy_tx_data = data_reg[bit_count_reg-1];
//                    state_next = STATE_WRITE_2;
//                end else begin
//                    // read ack bit
//                    phy_read_bit = 1'b1;
//                    state_next = STATE_WRITE_3;
//                end
//            end
//            STATE_WRITE_3: begin
//                // read ack bit
//                missed_ack_next = phy_rx_data_reg;
//
//                if (mode_write_multiple_reg && !last_reg) begin
//                    // more to write
//                    state_next = STATE_WRITE_1;
//                end else if (mode_stop_reg) begin
//                    // last cycle and stop selected
//                    phy_stop_bit = 1'b1;
//                    state_next = STATE_IDLE;
//                end else begin
//                    // otherwise, return to bus active state
//                    state_next = STATE_ACTIVE_WRITE;
//                end
//            end
//            STATE_READ: begin
//                // read data
//
//                bit_count_next = bit_count_reg - 1;
//                data_next = {data_reg[6:0], phy_rx_data_reg};
//                if (bit_count_reg > 0) begin
//                    // read next bit
//                    phy_read_bit = 1'b1;
//                    state_next = STATE_READ;
//                end else begin
//                    // output data word
//                    m_axis_data_tdata_next = data_next;
//                    m_axis_data_tvalid_next = 1'b1;
//                    m_axis_data_tlast_next = 1'b0;
//                    if (mode_stop_reg) begin
//                        // send nack and stop
//                        m_axis_data_tlast_next = 1'b1;
//                        phy_write_bit = 1'b1;
//                        phy_tx_data = 1'b1;
//                        state_next = STATE_STOP;
//                    end else begin
//                        // return to bus active state
//                        state_next = STATE_ACTIVE_READ;
//                    end
//                end
//            end
//            STATE_STOP: begin
//                // send stop bit
//                phy_stop_bit = 1'b1;
//                state_next = STATE_IDLE;
//            end
//        endcase
//    end
//end
//
//always @* begin
//    phy_state_next = PHY_STATE_IDLE;
//
//    phy_rx_data_next = phy_rx_data_reg;
//
//    delay_next = delay_reg;
//    delay_scl_next = delay_scl_reg;
//    delay_sda_next = delay_sda_reg;
//
//    scl_o_next = scl_o_reg;
//    sda_o_next = sda_o_reg;
//
//    bus_control_next = bus_control_reg;
//
//    if (phy_release_bus) begin
//        // release bus and return to idle state
//        sda_o_next = 1'b1;
//        scl_o_next = 1'b1;
//        delay_scl_next = 1'b0;
//        delay_sda_next = 1'b0;
//        delay_next = 1'b0;
//        phy_state_next = PHY_STATE_IDLE;
//    end else if (delay_scl_reg) begin
//        // wait for SCL to match command
//        delay_scl_next = scl_o_reg & ~scl_i_reg;
//        phy_state_next = phy_state_reg;
//    end else if (delay_sda_reg) begin
//        // wait for SDA to match command
//        delay_sda_next = sda_o_reg & ~sda_i_reg;
//        phy_state_next = phy_state_reg;
//    end else if (delay_reg > 0) begin
//        // time delay
//        delay_next = delay_reg - 1;
//        phy_state_next = phy_state_reg;
//    end else begin
//        case (phy_state_reg)
//            PHY_STATE_IDLE: begin
//                // bus idle - wait for start command
//                sda_o_next = 1'b1;
//                scl_o_next = 1'b1;
//                if (phy_start_bit) begin
//                    sda_o_next = 1'b0;
//                    delay_next = prescale;
//                    phy_state_next = PHY_STATE_START_1;
//                end else begin
//                    phy_state_next = PHY_STATE_IDLE;
//                end
//            end
//            PHY_STATE_ACTIVE: begin
//                // bus active
//                if (phy_start_bit) begin
//                    sda_o_next = 1'b1;
//                    delay_next = prescale;
//                    phy_state_next = PHY_STATE_REPEATED_START_1;
//                end else if (phy_write_bit) begin
//                    sda_o_next = phy_tx_data;
//                    delay_next = prescale;
//                    phy_state_next = PHY_STATE_WRITE_BIT_1;
//                end else if (phy_read_bit) begin
//                    sda_o_next = 1'b1;
//                    delay_next = prescale;
//                    phy_state_next = PHY_STATE_READ_BIT_1;
//                end else if (phy_stop_bit) begin
//                    sda_o_next = 1'b0;
//                    delay_next = prescale;
//                    phy_state_next = PHY_STATE_STOP_1;
//                end else begin
//                    phy_state_next = PHY_STATE_ACTIVE;
//                end
//            end
//            PHY_STATE_REPEATED_START_1: begin
//                // generate repeated start bit
//                //         ______
//                // sda XXX/      \_______
//                //            _______
//                // scl ______/       \___
//                //
//
//                scl_o_next = 1'b1;
//                delay_scl_next = 1'b1;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_REPEATED_START_2;
//            end
//            PHY_STATE_REPEATED_START_2: begin
//                // generate repeated start bit
//                //         ______
//                // sda XXX/      \_______
//                //            _______
//                // scl ______/       \___
//                //
//
//                sda_o_next = 1'b0;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_START_1;
//            end
//            PHY_STATE_START_1: begin
//                // generate start bit
//                //     ___
//                // sda    \_______
//                //     _______
//                // scl        \___
//                //
//
//                scl_o_next = 1'b0;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_START_2;
//            end
//            PHY_STATE_START_2: begin
//                // generate start bit
//                //     ___
//                // sda    \_______
//                //     _______
//                // scl        \___
//                //
//
//                bus_control_next = 1'b1;
//                phy_state_next = PHY_STATE_ACTIVE;
//            end
//            PHY_STATE_WRITE_BIT_1: begin
//                // write bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                scl_o_next = 1'b1;
//                delay_scl_next = 1'b1;
//                delay_next = prescale << 1;
//                phy_state_next = PHY_STATE_WRITE_BIT_2;
//            end
//            PHY_STATE_WRITE_BIT_2: begin
//                // write bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                scl_o_next = 1'b0;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_WRITE_BIT_3;
//            end
//            PHY_STATE_WRITE_BIT_3: begin
//                // write bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                phy_state_next = PHY_STATE_ACTIVE;
//            end
//            PHY_STATE_READ_BIT_1: begin
//                // read bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                scl_o_next = 1'b1;
//                delay_scl_next = 1'b1;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_READ_BIT_2;
//            end
//            PHY_STATE_READ_BIT_2: begin
//                // read bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                phy_rx_data_next = sda_i_reg;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_READ_BIT_3;
//            end
//            PHY_STATE_READ_BIT_3: begin
//                // read bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                scl_o_next = 1'b0;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_READ_BIT_4;
//            end
//            PHY_STATE_READ_BIT_4: begin
//                // read bit
//                //      ________
//                // sda X________X
//                //        ____
//                // scl __/    \__
//
//                phy_state_next = PHY_STATE_ACTIVE;
//            end
//            PHY_STATE_STOP_1: begin
//                // stop bit
//                //                 ___
//                // sda XXX\_______/
//                //             _______
//                // scl _______/
//
//                scl_o_next = 1'b1;
//                delay_scl_next = 1'b1;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_STOP_2;
//            end
//            PHY_STATE_STOP_2: begin
//                // stop bit
//                //                 ___
//                // sda XXX\_______/
//                //             _______
//                // scl _______/
//
//                sda_o_next = 1'b1;
//                delay_next = prescale;
//                phy_state_next = PHY_STATE_STOP_3;
//            end
//            PHY_STATE_STOP_3: begin
//                // stop bit
//                //                 ___
//                // sda XXX\_______/
//                //             _______
//                // scl _______/
//
//                bus_control_next = 1'b0;
//                phy_state_next = PHY_STATE_IDLE;
//            end
//        endcase
//    end
//end
//
//always @(posedge clk) begin
//    state_reg <= state_next;
//    phy_state_reg <= phy_state_next;
//
//    phy_rx_data_reg <= phy_rx_data_next;
//
//    addr_reg <= addr_next;
//    data_reg <= data_next;
//    last_reg <= last_next;
//
//    mode_read_reg <= mode_read_next;
//    mode_write_multiple_reg <= mode_write_multiple_next;
//    mode_stop_reg <= mode_stop_next;
//
//    delay_reg <= delay_next;
//    delay_scl_reg <= delay_scl_next;
//    delay_sda_reg <= delay_sda_next;
//
//    bit_count_reg <= bit_count_next;
//
//    s_axis_cmd_ready_reg <= s_axis_cmd_ready_next;
//
//    s_axis_data_tready_reg <= s_axis_data_tready_next;
//
//    m_axis_data_tdata_reg <= m_axis_data_tdata_next;
//    m_axis_data_tlast_reg <= m_axis_data_tlast_next;
//    m_axis_data_tvalid_reg <= m_axis_data_tvalid_next;
//
//    scl_i_reg <= scl_i;
//    sda_i_reg <= sda_i;
//
//    scl_o_reg <= scl_o_next;
//    sda_o_reg <= sda_o_next;
//
//    last_scl_i_reg <= scl_i_reg;
//    last_sda_i_reg <= sda_i_reg;
//
//    busy_reg <= !(state_reg == STATE_IDLE || state_reg == STATE_ACTIVE_WRITE || state_reg == STATE_ACTIVE_READ) || !(phy_state_reg == PHY_STATE_IDLE || phy_state_reg == PHY_STATE_ACTIVE);
//
//    if (start_bit) begin
//        bus_active_reg <= 1'b1;
//    end else if (stop_bit) begin
//        bus_active_reg <= 1'b0;
//    end else begin
//        bus_active_reg <= bus_active_reg;
//    end
//
//    bus_control_reg <= bus_control_next;
//    missed_ack_reg <= missed_ack_next;
//
//    if (rst) begin
//        state_reg <= STATE_IDLE;
//        phy_state_reg <= PHY_STATE_IDLE;
//        delay_reg <= 16'd0;
//        delay_scl_reg <= 1'b0;
//        delay_sda_reg <= 1'b0;
//        s_axis_cmd_ready_reg <= 1'b0;
//        s_axis_data_tready_reg <= 1'b0;
//        m_axis_data_tvalid_reg <= 1'b0;
//        scl_o_reg <= 1'b1;
//        sda_o_reg <= 1'b1;
//        busy_reg <= 1'b0;
//        bus_active_reg <= 1'b0;
//        bus_control_reg <= 1'b0;
//        missed_ack_reg <= 1'b0;
//    end
//end
//
//endmodule
//
//module i2c_master(
//		input wire clk,
//		input wire reset,
//		input wire start,
//		
//		input wire [7:0] nbytes_in,
//		input wire [6:0] addr_in,
//		input wire rw_in,
//		input wire [7:0] write_data,
//		output reg [7:0] read_data,
//		output reg tx_data_req, 
//		output reg rx_data_ready, 
//		
//		inout wire sda_w,
//		output wire scl
//	);
//	
//	//state parameters
//	localparam STATE_IDLE = 0;
//	localparam STATE_START = 1;
//	localparam STATE_ADDR = 2;
//	localparam STATE_RW = 3;
//	localparam STATE_ACK = 4;
//	localparam STATE_READ_ACK = 5;
//	localparam STATE_TX_DATA = 6;
//	localparam STATE_RX_DATA = 7;
//	localparam STATE_STOP = 8;
//	
//	localparam READ = 1;
//	localparam WRITE = 0;
//	localparam ACK = 0;
//	
//	reg [5:0] state;
//	reg [7:0] bit_count;	//bit counter
//	//local buffers
//	reg [6:0] addr;
//	reg [7:0] data;
//	reg [7:0] nbytes;
//	reg rw;
//	reg scl_en = 0;
//	reg sda;
//	
//	//i2c needs to float the sda line when it is high.
//	//so here I define sda_w which is the wire actually connected to the
//	// line to be z when sda (logical signal) is 1
//	assign sda_w = ((sda)==0) ? 1'b0 : 1'bz;
//	
//
//	//clock
//	//scl is enabled whenever we are sending or receiving data.
//	//  otherwise it is held at 1
//	//Note that I also ned to do an ACK check here on the negedge so that I am 
//	//  ready to respond on the next posedge below
//	assign scl = (scl_en == 0) ? 1'b1 : ~clk;
//	
//	always @(negedge clk) begin
//		if (reset == 1) begin
//			scl_en <= 0;
//			
//		end else begin
//			if ((state == STATE_IDLE) || (state == STATE_START) || (state == STATE_STOP)) begin
//				scl_en <= 0;
//			end
//			else begin
//				scl_en <= 1;
//			end
//			
//			//I need to check the ack on the rising scl edge (which is the neg edge of clk)
//			if (state == STATE_ACK) begin
//				if (0) begin
//					state <= STATE_IDLE;
//				end
//			end
//		end
//		
//	end
//	
//	
//
//	//FSM
//	always @(posedge clk) begin
//		if (reset == 1) begin
//			state <= STATE_IDLE;
//			sda <= 1;
//			bit_count <= 8'd0;
//			addr <= 0;
//			data <= 0;
//			nbytes <= 0;
//			rw <= 0;
//			tx_data_req <= 0;
//			rx_data_ready <= 0;
//		end	//if reset
//		
//		else begin
//			case(state)
//			
//				STATE_IDLE: begin	//idle
//					sda <= 1;
//					if (start) begin
//						state <= STATE_START;
//					end //if start
//				end
//				
//				
//				STATE_START: begin //start
//					state <= STATE_ADDR;
//					sda <= 0;	//send start condition
//					//latch in all the values
//					addr <= addr_in;
//					nbytes <= nbytes_in;
//					rw <= rw_in;
//					if (rw_in == WRITE) begin
//						tx_data_req <= 1;  //request the first byte of data
//					end
//					bit_count <= 6;	//addr is only 7 bits long, not 8
//				end	//state_start
//				
//				
//				STATE_ADDR: begin //send slave address
//					sda <= addr[bit_count];
//					if (bit_count == 0) begin
//						state <= STATE_RW;
//					end
//					else begin
//						bit_count <= bit_count - 1'b1;
//					end
//				end	//state_addr
//				
//				
//				STATE_RW: begin //send R/W bit
//					sda <= rw;
//					state <= STATE_ACK;
//				end	//state_rw
//				
//				
//				STATE_ACK: begin
//					//release the sda line and await ack
//					sda <= 1;
//					//Ack is checked on the next rising edge of scl (neg edge of clk)
//					//So I just assume that it is all ok and set the next state here
//					//if there is no ack then the state will be overwritten when it is checked
//					
//					tx_data_req <= 0; //time is up. if the data isn't in tx by now it is too late!
//					
//					//now we have to decide what to do next.
//					if (nbytes == 0) begin
//						//there is no data left to read/write
//						if (start == 1) begin
//							//repeat start condition
//							sda <= 1;
//							state <= STATE_START;
//						end else begin
//							//we are done
//							sda <= 1; //idle state is high
//							state <= STATE_STOP;
//						end	//if start == 1
//						
//					end else begin
//						//we have more data to read/write
//						if (rw == WRITE) begin
//							data <= write_data;  //latch in the new data byte
//							bit_count <= 7;  //8 data bits
//							state <= STATE_TX_DATA;
//						end else begin
//							// Read data
//							bit_count <= 7;	//8 data bits
//							state <= STATE_RX_DATA;
//						end //if rw_buf == WRITE
//					end //if nbytes_buf == 0
//						
//
//				end //state_ack
//				
//		
//
//				STATE_TX_DATA: begin
//					sda <= data[bit_count];
//					if (nbytes > 0) begin
//						tx_data_req <= 1;  //if there are more bytes to write, then request the next one
//					end
//					if (bit_count == 0) begin
//						//byte transfer complete
//						state <= STATE_ACK;
//						nbytes <= nbytes - 1'b1;
//					end
//					else begin
//						bit_count <= bit_count - 1'b1;
//					end
//				end	//state_tx_data
//				
//				
//				
//				STATE_RX_DATA: begin
//					data[bit_count] <= sda_w;
//					if (bit_count == 0) begin
//						//byte transfer complete
//						state <= STATE_ACK;
//						read_data[7:1] <= data[7:1];
//						read_data[0] <= sda_w;
//						rx_data_ready <= 1;
//						nbytes <= nbytes - 1'b1;
//					end
//					else begin
//						bit_count <= bit_count - 1'b1;
//						rx_data_ready <= 0;
//					end
//				end	//state_rx_data
//				
//				
//				
//				STATE_STOP: begin
//					sda <= 1;
//					state <= STATE_IDLE;
//				end	//state_stop
//				
//			endcase
//		end	//if reset (else)
//	end	//always
//
//endmodule
//
//module read_eeprom(
//	//inputs
//	input wire clk,
//	input wire reset,
//	input wire [6:0] slave_addr_w,
//	input wire [15:0] mem_addr_w,
//	input wire [7:0] read_nbytes_w,
//	input wire start,
//	
//	//outputs
//	output reg [7:0] data_out,
//	output reg byte_ready,
//	 
//	//i2c master comms lines
//	output reg [6:0] i2c_slave_addr,
//	output reg i2c_rw,
//	output reg [7:0] i2c_write_data,
//	output reg [7:0] i2c_nbytes,
//	input wire [7:0] i2c_read_data,
//	input wire i2c_tx_data_req,
//	input wire i2c_rx_data_ready,
//	output reg i2c_start,
//	output reg busy
//	);
//
//
//	//state params
//	localparam STATE_IDLE = 0;
//	localparam STATE_START = 1;
//	localparam STATE_WRITE_ADDR = 2;
//	localparam STATE_REP_START = 3;
//	localparam STATE_READ_DATA = 4;
//	
//	localparam READ = 1;
//	localparam WRITE = 0;
//	
//	//local buffers to save the transfer information (device slave addr, 
//	//  memory addr, etc) when the transfer is started
//	reg [3:0] state;
//	reg [6:0] slave_addr;
//	reg [15:0] mem_addr;
//	reg [7:0] read_nbytes; 
//	//output register definitions
//	reg waiting_for_tx;
//	reg read_prev_data;
//	reg [7:0] byte_count;
//	
//	
//	always @(posedge clk) begin
//	
//		if (reset == 1) begin
//			i2c_slave_addr <= 0;
//			i2c_rw <= 0;
//			i2c_write_data <= 0;
//			i2c_start <= 0;
//			i2c_nbytes <= 0;
//			
//			data_out <= 0;
//			byte_ready <= 0;
//			
//			mem_addr <= 0;
//			slave_addr <= 0;
//			read_nbytes <= 0;
//			byte_count <= 0;
//			waiting_for_tx <= 0;
//			
//			busy <= 0;
//			
//			state <= STATE_IDLE;
//			
//		end else begin
//		
//			case(state)
//			
//				STATE_IDLE: begin	//idle
//				
//					busy <= 0;	
//					
//					if (start) begin
//						state <= STATE_START;
//						
//						//buffer all the control data
//						slave_addr <= slave_addr_w;
//						mem_addr <= mem_addr_w;
//						read_nbytes <= read_nbytes_w;
//					end
//				end //state_idle
//				
//				
//				STATE_START: begin 
//					state <= STATE_WRITE_ADDR;
//					
//					//set all the i2c control lines
//					i2c_slave_addr <= slave_addr;
//					i2c_rw <= WRITE;
//					i2c_nbytes <= 2;  //2 memory addr bytes
//					byte_count <= 2;
//					waiting_for_tx <= 0;
//
//					i2c_start <= 1;
//					busy <= 1;
//				end //state_start
//				
//				
//				
//				STATE_WRITE_ADDR: begin
//					
//					if (waiting_for_tx == 0) begin
//						if (i2c_tx_data_req == 1) begin
//							waiting_for_tx <= 1;
//							case (byte_count)
//								2: begin
//									i2c_write_data <= mem_addr[15:8];
//									byte_count <= byte_count - 1'b1;
//								end //case 2
//								
//								1: begin
//									i2c_write_data <= mem_addr[7:0];
//									byte_count <= byte_count - 1'b1;
//									state <= STATE_REP_START;
//								end //case 1
//							endcase
//						end//if i2x_tx_data_req
//					end else begin
//						if (i2c_tx_data_req == 0) begin
//							waiting_for_tx <= 0;
//						end //if i2x_tx_data_req
//					end //if waiting_for_tx
//					
//				end //state WRITE_ADDR
//					
//
//					
//				STATE_REP_START: begin
//					state <= STATE_READ_DATA;
//					//set conditions for repeated start and change to read mode
//					i2c_start <= 1;
//					i2c_rw <= READ;
//					i2c_nbytes <= read_nbytes;
//					read_prev_data <= 0;
//					byte_count <= 0;
//					
//				end //state_rep_start
//
//				
//				
//				STATE_READ_DATA: begin
//				
//					if (read_prev_data == 0) begin
//						if (i2c_rx_data_ready) begin
//							data_out <= i2c_read_data;
//							byte_ready <= 1;
//							if (byte_count < (read_nbytes-1)) begin
//								byte_count <= byte_count + 1'b1;
//								read_prev_data <= 1;
//							end else begin
//								//we are done
//								i2c_start <= 0;
//								state <= STATE_IDLE;
//							end // if byte_count < read_nbytes
//						end //if i2c_rx_data_ready
//						
//					end else begin
//						if (i2c_rx_data_ready == 0) begin
//							read_prev_data <= 0;
//							byte_ready <= 0;
//						end //if i2c_rx_data_ready
//					end // if read_prev_data
//					
//				end //state_read_data
//			
//			endcase
//			
//		end
//	
//	end
//
//endmodule