module readwriteeeprom (SDA,SCL,clk,rst,RW,datain,dataout,address,done,trigger);
inout reg SDA;
output reg SCL;
input clk;
input rst;
input trigger; // pulse to start the process
input [11:0]address;
input [7:0] datain;
output [7:0] dataout;
output done;
parameter IDLE = 0,STARTR5=1,STARTW=2, sendaddressR = 3, sendaddressW = 4, senddata = 5, readdata = 6, stop = 7,restartread = 8,restartwrite = 9;
reg [7:0] counter;
reg [11:0] addressreg;
reg [7:0] datareg;
reg [7:0] dataoutreg;
parameter readeepromaddr = 3'b000;
parameter writeeepromaddr = 3'b001;
always @(posedge clk or negedge rst) begin
    if (!rst) begin
        counter <= 0;
        SCL <= 1; // SCL is high when idle
        SDA <= 1; // SDA is high when idle
    end
    else begin
        case (currentstate)
            IDLE:begin
                if(trigger)begin
                    datareg <= datain;
                    addressreg <= address;
                    counter <= 0;
                    if(RW == 0)begin
                        currentstate <= STARTW;
                    end
                    else begin
                        currentstate <= STARTR;
                    end
                end
                else 
                    currentstate <= IDLE;
            end
            STARTR:begin
                // send control byte wait ACK
                counter <= counter + 1;
                case(counter)
                0:begin
                    SCL <= 1;
                    SDA <= 1;
                end
                1:begin
                    SDA <= 0;
                    SCL <= 1;
                end
                2:begin
                    SDA <= 1;
                    SCL <= 0;
                end
                3:begin
                    SCA <= 1; // first control byte
                    SCL <= 0;
                end
                4:begin
                    SDA <= SDA; // cannot change SDA when SCL is high
                    SCL <= 1;
                end
                5:begin
                    SDA <= SDA; // cannot change SDA when SCL is high
                    SCL <= 1;
                end
                6:begin
                    SDA <= 0;// second control byte
                    SCL <= 0;
                end
                7:begin
                    SDA <= SDA; // cannot change SDA when SCL is high
                    SCL <= 0;
                end
                8:begin
                    SDA <= SDA; // cannot change SDA when SCL is high
                    SCL <= 1;
                end
                9:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                10:begin
                    SDA <= 1; // third control byte
                    SCL <= 0;
                end
                11:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                12:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                13:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                14:begin
                    SDA <= 0; // fourth control byte
                    SCL <= 0;
                end
                15:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                16:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                17:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                18:begin
                    SDA <= readeepromaddr[2]; // address byte A2 
                    SCL <= 0;
                end
                19:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                20:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                21:begin
                    SDA <=SDA; 
                    SCL <= 1;
                end
                22:begin
                    SDA <= readeepromaddr[1]; // address byte A1
                    SCL <= 0;
                end
                23:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                24:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                25:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                26:begin
                    SDA <= readeepromaddr[0]; // address byte A0
                    SCL <= 0;
                end
                27:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                28:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                29:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                30:begin
                    SDA <= 0; // random read need to write address first 
                    SCL <= 0;
                end
                31:begin
                    SDA <= SDA;
                    SCL <= 0;
                end
                32:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                33:begin
                    SDA <= SDA;
                    SCL <= 1;
                end
                34:begin
                    SDA <= 8'hZZ; // high impedance to read if ACK is received
                    SCL <= 0;
                end
                35:begin
                    SDA <= 8'hZZ;
                    SCL <= 0;
                end
                36:begin
                    SDA <= 8'hZZ;
                    SCL <= 1;
                end
                37:begin
                    if(SDA == 0)begin
                        // ACK received
                        sendaddressR <= 1;
                        counter <= 0;
                    end
                    else begin
                        // NACK received
                        counter <= 0;
                    end
                end
                endcase
            end
            default:; 
        endcase
    end
end

endmodule

// old eh stuff
//wire i2c_start;
//wire i2c_reset;
//wire [7:0] i2c_nbytes;
//wire [6:0] i2c_slave_addr;
//wire i2c_rw;
//wire [7:0] i2c_write_data;
//wire [7:0] i2c_read_data;
//wire i2c_tx_data_req;
//wire i2c_rx_data_ready;
//wire i2c_sda;
//wire i2c_scl;
//wire ready;
//wire busy;
//i2c_master i2c (
//	.clk(clock400khz), 
//	.reset(i2c_reset), 
//	.start(i2c_start), 
//	.nbytes_in(i2c_nbytes), 
//	.addr_in(i2c_slave_addr), 
//	.rw_in(i2c_rw), 
//	.write_data(i2c_write_data), 
//	.read_data(i2c_read_data), 
//	.tx_data_req(i2c_tx_data_req), 
//	.rx_data_ready(i2c_rx_data_ready), 
//	.sda_w(EEP_I2C_SDAT), 
//	.scl(EEP_I2C_SCLK)
//);
//reg [6:0] slave_addr = 7'b1010000;
//reg [15:0] mem_addr = 16'h0000;
//
//reg [7:0] read_nbytes; // number of bytes to read
//reg start; // start reading 
//wire [7:0] data_out;
//wire byte_ready; // data is ready to be read
//reg [63:0]delayco;
//always @(posedge clk or negedge rst) begin
//    if(!rst)begin
//        slave_addr <= 7'b1010000;
//        mem_addr <= 16'h0000;
//        read_nbytes <= 8'h02;
//        start <= 1'b0;
//        delayco <= 0;
//    end
//    else begin
//        delayco <= delayco + 1'b1;
//        if(enable && delayco > 10000000)begin
//            delayco <= 0;
//            if(!busy)begin
//                if(KEY[1])begin
//                    start <= 1'b1;
//                    mem_addr <= mem_addr + 1'b1;
//                end
//                else begin
//                end
//            end
//        end
//        else 
//            start <= 1'b0;
//    end
//end
//read_eeprom instance_name (
//    .clk(clk), 
//    .reset(!rst), 
//    .slave_addr_w(slave_addr), 
//    .mem_addr_w(mem_addr), 
//    .read_nbytes_w(read_nbytes), 
//    .start(start), 
//    .data_out(data_out), 
//    .byte_ready(byte_ready), 
//    .i2c_slave_addr(i2c_slave_addr), 
//    .i2c_rw(i2c_rw), 
//    .i2c_write_data(i2c_write_data), 
//    .i2c_nbytes(i2c_nbytes), 
//    .i2c_read_data(i2c_read_data), 
//    .i2c_tx_data_req(i2c_tx_data_req), 
//    .i2c_rx_data_ready(i2c_rx_data_ready), 
//    .i2c_start(i2c_start)
//);
//
//
//
//? old
//reg [7:0]voltage;
//reg direction=1;  
//reg SDA;
//wire scl;
//assign scl = EEP_I2C_SCLK;
//assign EEP_I2C_SDAT =direction?SDA:1'bZ;   
//assign LEDR[17:10] = state;
//reg [7:0] state; 
//reg [7:0] address;                  // ADDRESS including read bit
//reg [7:0] address2;
//reg [7:0] count;
//reg ack;
//reg received_bit;
//parameter   STATE_IDLE=0,
//            STATE_START=1,
//            STATE_ADD_RW=2,
//            STATE_ACK=3,
//            STATE_DATA=4,
//            STATE_RACK=5,
//            STATE_STOP=6,
//            STATE_WRITEADDR1=7,
//            STATE_WRITEADDR1ACK=8,
//            STATE_WRITEADDR2=9,
//            STATE_WRITEADDR2ACK=10,
//            STATE_STARTWRITE=11,// basically same as state start>?
//            STATE_ADD_RW2=12,
//            STATE_ACK2=13; // go to STATE_DATA
//always@(negedge clock200khz)
//begin
//    if(rst==0)  EEP_I2C_SCLK<=1;
//    else begin
//             if((state==STATE_IDLE)||(state==STATE_START)||(state==STATE_STOP))  
//                     EEP_I2C_SCLK<=1;
//            else    EEP_I2C_SCLK<=~EEP_I2C_SCLK;
//         end 
//end 
//always@(posedge clock200khz or negedge rst)
//begin
//    if(!rst) begin
//        state<=STATE_IDLE;
//        direction<=1;
//        SDA<=1;
//        address=8'b10100000;
//        address2=8'b10100001;
//        count<=8'd0;      
//       end
//    else begin
//        case(state)
//        STATE_IDLE:  begin                     //idle
//                        direction<=1;
//                        SDA<=1;
//                        if(!enable)
//                            state<=STATE_START;
//                     end
//        STATE_START: begin                       //start
//                        SDA<=0;
//                        count<=8;
//                        state<=STATE_ADD_RW;
//                    end
//        STATE_ADD_RW: begin                       //address
//                        if(EEP_I2C_SCLK==0) begin
//                        SDA<=address[count-1];
//                        if(count==0)  begin  state<=STATE_ACK; direction<=0;   end
//                        else count<=count-1;
//                        end
//        end
//        STATE_ACK:    begin                       //acknowledge by slave
//                        //ack<=EEP_I2C_SDAT; // for real time
//                        if(EEP_I2C_SDAT==0) begin  
//                            state<=STATE_WRITEADDR1; 
//                            count<=8; 
//                            direction <= 1; //after ack i wan to write
//                        end
//                        else begin  
//                            state<=STATE_IDLE;   
//                        end
//                     end
//        STATE_WRITEADDR1: begin                        //start to receive data by slave
//                        if(EEP_I2C_SCLK==0) begin           //data is received only when scl is low
//                            SDA <= 0; // all zero for now cause i want to write to address 0
//                            count<=count-1'b1;
//                            if(count==0) begin     
//                                state<=STATE_WRITEADDR1ACK;
//                                direction<=0;// is read           
//                                end
//                            else begin    // vtg<=vtg+1'b1;  // for bug test
//                                state<=STATE_WRITEADDR1;
//                        end
//                    end
//        end
//        STATE_WRITEADDR1ACK: begin             //acknowledge by slave
//                        //ack<=EEP_I2C_SDAT; // for real time
//                        // if ack then go to next state
//                        if(EEP_I2C_SDAT==0) begin
//                            state<=STATE_WRITEADDR2;
//                            count<=8;
//                            direction<=1; // after ack i want to write
//                            end
//                        else    begin  state<=STATE_IDLE;   end
//                     end
//        STATE_WRITEADDR2:begin
//                        if(EEP_I2C_SCLK==0) begin           //data is received only when scl is low
//                            SDA <= 0; // all zero for now cause i want to write to address 0
//                            count<=count-1'b1;
//                            if(count==0) begin     
//                                state<=STATE_WRITEADDR2ACK;
//                                direction<=0;// is read           
//                                end
//                            else begin    // vtg<=vtg+1'b1;  // for bug test
//                                state<=STATE_WRITEADDR2;
//                        end
//                    end
//        end 
//        STATE_WRITEADDR2ACK:begin
//                        if(EEP_I2C_SDAT==0) begin
//                            state<=STATE_STARTWRITE;
//                            //count<=8;
//                            direction<=1; // after ack i want to write
//                            SDA <= 1;
//                            end
//                        else    begin  state<=STATE_IDLE;   end
//        end
//        STATE_STARTWRITE:begin
//            if(EEP_I2C_SCLK == 1)begin
//                SDA <= 0;
//                count <= 8;
//                state <= STATE_ADD_RW2;
//                direction <= 1; // i want to write
//            end
//        end
//        STATE_ADD_RW2:begin
//            if(EEP_I2C_SCLK==0) begin
//                SDA<=address2[count-1];
//                if(count==0)  begin  state<=STATE_ACK2; direction<=0;   end
//                else count<=count-1;
//            end
//        end
//        STATE_ACK2:begin
//            if(EEP_I2C_SDAT==0) begin
//                state<=STATE_DATA;
//                count<=8;
//                direction<=0; // still read
//            end
//            else begin
//                state<=STATE_IDLE;
//            end
//        end
//        STATE_DATA:  begin                        //start to receive data by slave
//                         if(EEP_I2C_SCLK==0) begin           //data is received only when scl is low
//                          received_bit<=EEP_I2C_SDAT;
//                          voltage[0]<=received_bit;
//                          voltage<=voltage<<1'b1;
//                          count<=count-1'b1;
//                          if(count==0) begin     state<=STATE_RACK;
//                                                 direction<=1; SDA<=1;      //actual ack
//                                                 //vtg<=8'd15; //vtg<=voltage for real time
//                                       end
//                          else    begin    // vtg<=vtg+1'b1;  // for bug test
//                                            state<=STATE_DATA;   end
//                    end
//                    end
//       STATE_RACK:  begin                        //master acknowledging the data sent 
//                       if(EEP_I2C_SCLK==0) begin
//                        SDA<=0;             //making the line zero so that SDA toggles next when scl is one for stop
//                        state<=STATE_STOP;
//                    end
//                    end
//       STATE_STOP: begin                         //stop
//                        SDA<=1;
//                        state<=STATE_IDLE;
//                   end
//        endcase              
//       end
//    end
/*
reg [6:0]  s_axis_cmd_address;
reg        s_axis_cmd_start;
reg        s_axis_cmd_read;
reg        s_axis_cmd_write;
reg        s_axis_cmd_write_multiple;
reg        s_axis_cmd_stop;
reg        s_axis_cmd_valid;
wire        s_axis_cmd_ready;
reg [7:0]  s_axis_data_tdata; // data to be sent1
reg        s_axis_data_tvalid;
wire        s_axis_data_tready;
reg        s_axis_data_tlast;
wire [7:0]  m_axis_data_tdata; // received data
wire        m_axis_data_tvalid;
reg        m_axis_data_tready;
wire        m_axis_data_tlast;
wire        scl_i;
wire        scl_o;
wire        scl_t;
wire        sda_i;
wire        sda_o;
wire        sda_t;
wire        busy;
wire        bus_control;
wire        bus_active;
wire        missed_ack;
wire        stop_on_idle;
assign LEDR[13] = busy;
assign LEDR[12] = bus_control;
assign LEDR[11] = bus_active;
assign LEDR[10] = missed_ack;
assign LEDR[9] = s_axis_cmd_ready;
assign stop_on_idle = 1'b1;
assign scl_i = EEP_I2C_SCLK;
assign EEP_I2C_SCLK = scl_t ? 1'bz : scl_o;
assign sda_i = EEP_I2C_SDAT;
assign EEP_I2C_SDAT = sda_t ? 1'bz : sda_o;
i2c_master i2c(.clk(clk),.rst(!rst),.s_axis_cmd_address(s_axis_cmd_address),.s_axis_cmd_start(s_axis_cmd_start),.s_axis_cmd_read(s_axis_cmd_read),.s_axis_cmd_write(s_axis_cmd_write),.s_axis_cmd_write_multiple(s_axis_cmd_write_multiple),.s_axis_cmd_stop(s_axis_cmd_stop),.s_axis_cmd_valid(s_axis_cmd_valid),.s_axis_cmd_ready(s_axis_cmd_ready),.s_axis_data_tdata(s_axis_data_tdata),.s_axis_data_tvalid(s_axis_data_tvalid),.s_axis_data_tready(s_axis_data_tready),.s_axis_data_tlast(s_axis_data_tlast),.m_axis_data_tdata(m_axis_data_tdata),.m_axis_data_tvalid(m_axis_data_tvalid),.m_axis_data_tready(m_axis_data_tready),.m_axis_data_tlast(m_axis_data_tlast),.scl_i(scl_i),.scl_o(scl_o),.scl_t(scl_t),.sda_i(sda_i),.sda_o(sda_o),.sda_t(sda_t),.busy(busy),.bus_control(bus_control),.bus_active(bus_active),.missed_ack(missed_ack),.stop_on_idle(stop_on_idle),.prescale(prescale));
reg [7:0] eepromcounter;
assign LEDR[17:14] = eepromcounter; // to know which state we stuck in if we stuck
always @(posedge clk) begin
    //try to do a simple read
    // A0 for writing and A1 for reading
    // we write using A0 address first then we read out using A1 address 
    if(!rst)begin
        s_axis_cmd_address <= 7'h50;
        s_axis_cmd_start <= 1'b0;
        s_axis_cmd_read <= 1'b0;
        s_axis_cmd_write <= 1'b0;
        s_axis_cmd_write_multiple <= 1'b0;
        s_axis_cmd_stop <= 1'b0;
        s_axis_cmd_valid <= 1'b0;
        s_axis_data_tvalid <= 1'b0;
        s_axis_data_tlast <= 1'b0;
        m_axis_data_tready <= 1'b0;
        eepromcounter <= 0;
        dataiwant <= 8'h00;
        dataiwant2 <= 8'h00;
    end
    else begin
        case(eepromcounter)
        0:begin
            // first we write the address we want
            if(enable)
                if(s_axis_cmd_ready)begin
                    m_axis_data_tready <= 1'b1;
                    s_axis_cmd_address <= 7'h50;
                    s_axis_data_tdata <= 8'h00;
                    s_axis_cmd_read <= 1'b0;
                    s_axis_cmd_write <= 1'b0;
                    s_axis_cmd_write_multiple <= 1'b1;
                    s_axis_data_tlast <= 1'b0;
                    s_axis_cmd_stop <= 1'b1;
                    s_axis_cmd_valid <= 1'b1;
                    s_axis_data_tvalid <= 1'b1;
                    eepromcounter <= 1;
                end
        end
        1:begin
            // not used forgot it send first data on top automatically
            s_axis_cmd_valid <= 1'b0; // stop valid 
            if(s_axis_data_tready)begin // first data out 
                s_axis_data_tdata <= 8'h00;
                s_axis_data_tvalid <= 1'b1;
                s_axis_data_tlast <= 1'b0;
                eepromcounter <= 2;
            end 
        end
        2:begin
            if(s_axis_data_tready)begin
                s_axis_data_tdata <= 8'h01;//read address 1
                s_axis_data_tlast <= 1'b1;
                s_axis_data_tvalid <= 1'b1;
                s_axis_cmd_read <= 1'b0;
                s_axis_cmd_write <= 1'b0;
                s_axis_cmd_write_multiple <= 1'b0;// do not touch this
                eepromcounter <= 3;
            end
                else 
                    eepromcounter <= 2;
        end
        // wait cmd to be ready
        3:begin
            if(!busy)begin
                s_axis_data_tlast <= 1'b0;
                s_axis_data_tvalid <= 1'b0;
                eepromcounter <= 4;
            end
            else 
                eepromcounter <=3;
        end
        4:begin
            // now we read the data
            if(s_axis_cmd_ready == 1'b1)begin
                s_axis_cmd_address <= 8'h51;
                s_axis_cmd_write_multiple <= 1'b0;
                s_axis_cmd_read <= 1'b1;
                s_axis_cmd_write <= 1'b0;
                s_axis_cmd_valid <= 1'b1;
                s_axis_cmd_stop <= 1'b0;
                eepromcounter <= 5;
            end
            else 
                eepromcounter <= 4;
        end
        5:begin
            // wait for the data to be valid
            if(m_axis_data_tvalid)begin
                dataiwant <= m_axis_data_tdata;
                m_axis_data_tready <= 1'b1;
                eepromcounter <= 6;
                s_axis_cmd_valid <= 1'b0;
            end
        end
        6:begin
            //stop everything
            if(m_axis_data_tvalid)begin
                dataiwant2 <= m_axis_data_tdata;
                m_axis_data_tready <= 1'b1;
                eepromcounter <= 0;
            end
            else if(s_axis_cmd_ready)begin
                eepromcounter <= 0;
            end
        end
        endcase
    end
end  */

//reg [7:0] wb_dat_i; // data in
//wire [7:0] wb_dat_o; // data out
//reg [0:0] wb_we_i; // write enable 
//reg [0:0] wb_stb_i; // strobe
//reg [0:0] wb_cyc_i; // valid cycle
//wire [0:0] wb_ack_o; // bus cycle ack
//wire [0:0] wb_inta_o; // interrupt
//wire scl_padoen_oe; wire sda_padoen_oe;
//assign EEP_I2C_SCLK = scl_padoen_oe ? 1'bz : scl_pad_o; 
//assign EEP_I2C_SDAT = sda_padoen_oe ? 1'bz: sda_pad_o;
//wire scl_pad_o; wire sda_pad_o;
//wire scl_pad_i; wire sda_pad_i;
//assign scl_pad_i = EEP_I2C_SCLK; assign sda_pad_i = EEP_I2C_SDAT; 
//
//i2c_master_top i2cmaster(clk, !rst, 1'b0, wb_adr_i, wb_dat_i, wb_dat_o,
//	wb_we_i, wb_stb_i, wb_cyc_i, wb_ack_o, wb_inta_o,
//	scl_pad_i, scl_pad_o, scl_padoen_oe, sda_pad_i, sda_pad_o, sda_padoen_oe 
//    );
//reg [7:0]i2cstatemachine;
//assign LEDR[17:10] = i2cstatemachine;
//parameter transmitreceivereg = 3'b011; // when write then it will be transmit and when read it will be receive
//parameter commandreg = 3'b100;
//// simple state machine to control the i2c
//always @(posedge clk or negedge rst) begin
//    if(!rst)begin
//        wb_dat_i <= 0;
//        wb_we_i <= 0;
//        wb_stb_i <= 0;
//        wb_cyc_i <= 0;
//        wb_adr_i <= 0;
//        i2cstatemachine <= 0;
//    end 
//    else begin
//        case(i2cstatemachine)
//        0:begin
//            if(KEY[3] == 0)begin
//            // set prescale first
//            // i wait for ACK before sending another byte
//            wb_adr_i <= 3'b000; // address of prescale low byte
//            wb_dat_i <= 8'd24; // 24 for 400khz
//            wb_we_i <= 1'b1; // 1 = write
//            wb_stb_i <= 1'b1;
//            wb_cyc_i <= 1'b1;
//            if(wb_ack_o == 1'b1)begin
//                wb_stb_i <= 1'b0;
//                wb_cyc_i <= 1'b0;
//                i2cstatemachine <= 1;
//            end
//            end
//        end
//        1:begin
//            wb_adr_i <= 3'b001; // address of prescale high byte
//            wb_dat_i <= 8'd0; // 0 for 400khz
//            wb_we_i <= 1'b1; // 1 = write
//            wb_stb_i <= 1'b1;
//            wb_cyc_i <= 1'b1;
//            if(wb_ack_o == 1'b1)begin
//                wb_stb_i <= 1'b0;
//                wb_cyc_i <= 1'b0;
//                i2cstatemachine <= 2;
//            end
//        end
//        2:begin
//            wb_adr_i <= 3'b010; // address of control register
//            wb_dat_i <= 8'b10000000; // 1 = enable i2c
//            wb_we_i <= 1'b1; // 1 = write
//            wb_stb_i <= 1'b1;
//            wb_cyc_i <= 1'b1;
//            if(wb_ack_o == 1'b1)begin
//                wb_stb_i <= 1'b0;
//                wb_cyc_i <= 1'b0;
//                i2cstatemachine <= 3;
//            end
//        end
//        3:begin
//            wb_adr_i <= transmitreceivereg;
//            wb_dat_i <= 8'b10100000; // setup for write just to write address
//            wb_we_i <= 1'b1; // 1 = write
//            wb_stb_i <= 1'b1;
//            wb_cyc_i <= 1'b1;
//            if(wb_ack_o == 1'b1)begin
//                wb_stb_i <= 1'b0;
//                wb_cyc_i <= 1'b0;
//                i2cstatemachine <= 4;
//            end
//        end
//        4:begin
//            wb_adr_i <= commandreg;
//            wb_dat_i <= 8'b10010000;// generate start condition and write
//            wb_we_i <= 1'b1; // 1 = write
//            wb_stb_i <= 1'b1;
//            wb_cyc_i <= 1'b1;
//            if(wb_ack_o == 1'b1)begin
//                i2cstatemachine <= 5;
//                wb_we_i <= 1'b0;
//                wb_adr_i <= commandreg;
//            end
//        end
//        5:begin
//            // keep reading until ack received
//            // wait until ack received
//            if(wb_dat_o[1] == 0) // wait for TIP to be 0
//            begin
//                if(wb_dat_o[7] == 0)begin
//                wb_dat_i <= 8'b00000000; // prepare for next byte
//                wb_we_i <= 1'b1; 
//                wb_adr_i <= transmitreceivereg;
//                wb_stb_i <= 1'b1;
//                wb_cyc_i <= 1'b1;
//                i2cstatemachine <= 6;
//                end
//            end
//        end
//        6:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_dat_i <= 8'b00010000; // send address of eeprom 
//                wb_we_i <= 1'b1; 
//                wb_adr_i <= commandreg;
//                wb_stb_i <= 1'b1;
//                wb_cyc_i <= 1'b1;
//                i2cstatemachine <= 7;
//            end
//        end
//        7:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_we_i <= 1'b0;
//                // read until ack received
//                i2cstatemachine <= 8;
//            end
//        end
//        8:begin
//            if(wb_dat_o[1] == 0) // wait for TIP to be 0
//            begin
//                if(wb_dat_o[7] == 0)begin
//                wb_dat_i <= 8'b00010000; // send address again so both is 0
//                wb_we_i <= 1'b1; 
//                wb_adr_i <= commandreg;
//                wb_stb_i <= 1'b1;
//                wb_cyc_i <= 1'b1;
//                i2cstatemachine <= 9;
//                end
//            end
//        end
//        9:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_we_i <= 1'b0;
//                // read until ack received
//                i2cstatemachine <= 10;
//            end
//        end
//        10:begin
//            if(wb_dat_o[1] == 0)begin
//                if(wb_dat_o[7] == 0)begin
//                // send a start condition with read bit
//                wb_dat_i <= 8'b10100001; // send address again so both is 0
//                wb_we_i <= 1'b1;
//                wb_adr_i <= transmitreceivereg;
//                wb_stb_i <= 1'b1;
//                wb_cyc_i <= 1'b1;
//                i2cstatemachine <= 11;
//                end
//            end
//        end
//        11:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_we_i <= 1'b1;
//                wb_adr_i <= commandreg;
//                wb_dat_i <= 8'b10010000; // generate start condition and write
//                // read until ack received
//                i2cstatemachine <= 12;
//            end
//        end
//        12:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_we_i <= 1'b0;
//                // read until ack received
//                i2cstatemachine <= 13;
//            end
//        end
//        13:begin
//            if(wb_dat_o[1] == 0)begin
//                if(wb_dat_o[7] == 0)begin
//                wb_we_i <= 1'b1;
//                wb_adr_i <= commandreg;
//                wb_dat_i <= 8'b00100000; // set read bit
//                i2cstatemachine <= 14;
//                end
//            end
//        end
//        14:begin
//            if(wb_ack_o == 1'b1)begin
//                wb_we_i <= 1'b0;
//                // read until ack received
//                i2cstatemachine <= 15;
//            end
//        end
//        15:begin
//            if(wb_dat_o[1] == 0)begin
//                wb_adr_i <= transmitreceivereg;
//                wb_we_i <= 1'b0; // read mode
//                i2cstatemachine <= 16;
//            end
//        end
//        16:begin
//            i2cstatemachine <= 16; //stuck here forever
//            dataiwant <= wb_dat_o;
//        end
//        endcase
//    end   
//end
// 00 Prescale low byte
// prescale is 24 for 400khz
// 01 Prescale high byte
// 02 Control register bit 7 set to 1 to enable I2C
// 03 Transmit register 7:1 next byte to send then bit 0 is read/write if not byte send
// 04 Receive register last byte received
// 05 Command register
// bit 7 W STA, generate (repeated) start condition 
// bit 6 W STO, generate stop condition 
// bit 5 W RD, read from slave 
// bit 4 W WR, write to slave 
// bit 3 W ACK, when a receiver, sent ACK (ACK = ‘0’) or NACK (ACK = ‘1’) 
// bit 2:1 W Reserved 
// bit 0 W IACK, Interrupt acknowledge. When set, clears a pending interrupt.
// 06 Status register
//bit7 RxACK, Received acknowledge from slave. 
//      This flag represents acknowledge from the addressed slave. 
//      ‘1’ = No acknowledge received 
//      ‘0’ = Acknowledge received 
//bit6 Busy, I2C bus busy 
//      ‘1’ after START signal detected 
//      ‘0’ after STOP signal detected 
//bit5 AL, Arbitration lost 
//  This bit is set when the core lost arbitration. Arbitration is lost when: 
//      • a STOP signal is detected, but non requested 
//      • The master drives SDA high, but SDA is low. 
//bit4:2 Reserved 
//bit1 TIP, Transfer in progress. 
//      ‘1’ when transferring data 
//      ‘0’ when transfer complete 
//bit0 IF, Interrupt Flag. This bit is set when an interrupt is pending, which 
//      will cause a processor interrupt request if the IEN bit is set. 
//      The Interrupt Flag is set when: 
//      • one byte transfer has been completed 
//      • arbitration is lost
