module i2c_eeprom (
    input            clk,
    input            rst_n,
    inout            EEP_I2C_SDAT,
    inout            EEP_I2C_SCLK,
    input [7:0] data_in,
    input enable,
    input LreadHwrite,
    output reg valid,
    output reg [7:0] data_out,
    input [15:0] addr_in // will store to register when write / read
    );
localparam S_IDLE       = 0;
localparam S_READ       = 1;
localparam S_WAIT       = 2;
localparam S_WRITE      = 3;

reg[3:0] state;
reg[7:0] read_data;
reg[31:0] timer;

wire scl_pad_i;
wire scl_pad_o;
wire scl_padoen_o;

wire sda_pad_i;
wire sda_pad_o;
wire sda_padoen_o;

reg[15:0] i2c_slave_reg_addr;
reg[ 7:0] i2c_write_data;
reg i2c_read_req;
wire i2c_read_req_ack;
reg i2c_write_req;
wire i2c_write_req_ack;
wire[7:0] i2c_read_data;
reg [7:0]i2c_slave_dev_addr;

always@(posedge clk or negedge rst_n)
begin
    if(rst_n == 1'b0) //if(!rst_n)
    begin
        state <= S_IDLE;
        i2c_write_req <= 1'b0;
        read_data <= 8'h00;
        timer <= 32'd0;
        i2c_write_data <= 8'd0;
        valid <= 1'b0;
        i2c_slave_dev_addr <= 8'hA0;//1010 000 0(default address ‘000’ write operation)
        i2c_read_req <= 1'b0;
    end
    else
        case(state)
            S_IDLE:
            begin
                if(timer >= 32'd12_499_999)//250ms
                    state <= S_READ;
                else
                    timer <= timer + 32'd1;
            end
            S_READ:
            begin
                if(i2c_read_req_ack)
                begin
                    i2c_read_req <= 1'b0;
                    data_out <= i2c_read_data;
                    state <= S_WAIT;
                end
                else
                begin
                    i2c_read_req <= 1'b1;
                    i2c_slave_dev_addr <= 8'ha0;
                end
            end
            S_WAIT:
            begin
                if(enable)begin
                    if(LreadHwrite)
                    begin
                        state <= S_WRITE;
                        i2c_slave_reg_addr <= addr_in;
                        valid <= 1'b0;
                    end
                    else begin
                        state <= S_READ;
                        i2c_slave_reg_addr <= addr_in;
                        valid <= 1'b0;
                    end
                end
                else begin
                    state <= S_WAIT;
                    valid <= 1'b1;
                end
            end
            S_WRITE:
            begin
                if(i2c_write_req_ack)
                begin
                    i2c_write_req <= 1'b0;
                    state <= S_READ;
                end
                else
                begin
                    i2c_write_req <= 1'b1;
                    i2c_write_data <= data_in;
                end
            end
            
            default:
                    state <= S_IDLE;
        endcase
end

assign sda_pad_i = EEP_I2C_SDAT;
assign EEP_I2C_SDAT = ~sda_padoen_o ? sda_pad_o : 1'bz;
assign scl_pad_i = EEP_I2C_SCLK;
assign EEP_I2C_SCLK = ~scl_padoen_o ? scl_pad_o : 1'bz;

i2c_master_top i2c_master_top_m0
(
    .rst          (~rst_n      ),
    .clk          (clk         ),
    .clk_div_cnt  (16'd500     ),       //Standard mode:100Khz
    
    // I2C signals 
    // i2c clock line
    .scl_pad_i    (scl_pad_i   ),            // SCL-line input
    .scl_pad_o    (scl_pad_o   ),            // SCL-line output (always 1'b0)
    .scl_padoen_o (scl_padoen_o),      // SCL-line output enable (active low)

    // i2c data line
    .sda_pad_i    (sda_pad_i   ),           // SDA-line input
    .sda_pad_o    (sda_pad_o   ),           // SDA-line output (always 1'b0)
    .sda_padoen_o (sda_padoen_o),     // SDA-line output enable (active low)
    
    
    .i2c_addr_2byte      (1'b1              ),
    .i2c_read_req        (i2c_read_req      ),
    .i2c_read_req_ack    (i2c_read_req_ack  ),
    .i2c_write_req       (i2c_write_req     ),
    .i2c_write_req_ack   (i2c_write_req_ack ),
    .i2c_slave_dev_addr  (i2c_slave_dev_addr),
    .i2c_slave_reg_addr  (i2c_slave_reg_addr),
    .i2c_write_data      (i2c_write_data    ),
    .i2c_read_data       (i2c_read_data     ),
    .error               (                  )
);
endmodule