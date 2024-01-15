`define I2C_CMD_NOP   4'b0000
`define I2C_CMD_START 4'b0001
`define I2C_CMD_STOP  4'b0010
`define I2C_CMD_WRITE 4'b0100
`define I2C_CMD_READ  4'b1000

module i2c_master_top
(
	input rst,
	input clk,
	input[15:0] clk_div_cnt, //This register is used to prescale the SCL clock line. Due to the structure of the I2C
							       //interface, the core uses a 5*SCL clock internally. The prescale register must be
							       //programmed to this 5*SCL frequency (minus 1); 50Mhz/(5*100Khz) - 1 = 99;
							       //50Mhz/(5*400Khz) - 1 = 24;
	
	// I2C signals
	// i2c clock line
	input  scl_pad_i,                           // SCL-line input
	output scl_pad_o,                           // SCL-line output (always 1'b0)
	output scl_padoen_o,                        // SCL-line output enable (active low)
	// i2c data line                            
	input  sda_pad_i,                           // SDA-line input
	output sda_pad_o,                           // SDA-line output (always 1'b0)
	output sda_padoen_o,                        // SDA-line output enable (active low)
	
	input  i2c_addr_2byte,                      // Is the register address 16bit?
	input  i2c_read_req,                        // Read register request
	output i2c_read_req_ack,                    // Read register request response
	input  i2c_write_req,                       // Write register request
	output i2c_write_req_ack,                   // Write register request response
	input[7:0] i2c_slave_dev_addr,              // I2c device address
	input[15:0] i2c_slave_reg_addr,             // I2c register address
	input[7:0] i2c_write_data,                  // I2c write register data
	output reg[7:0] i2c_read_data,              // I2c read register data
	output reg error                            // The error indication, generally there is no response
);
//State machine definition
localparam S_IDLE             =  0;             // Idle state, waiting for read and write
localparam S_WR_DEV_ADDR      =  1;             // Write device address
localparam S_WR_REG_ADDR      =  2;             // Write register address 
localparam S_WR_DATA          =  3;             // Write register data
localparam S_WR_ACK           =  4;             // Write request response
localparam S_WR_ERR_NACK      =  5;             // Write error, I2C device is not responding
localparam S_RD_DEV_ADDR0     =  6;             // I2C read state, first writes the device address and the register address
localparam S_RD_REG_ADDR      =  7;             // I2C read state, read register address (8bit)
localparam S_RD_DEV_ADDR1     =  8;             // Write the device address again
localparam S_RD_DATA          =  9;             // Read data
localparam S_RD_STOP          = 10;  
localparam S_WR_STOP          = 11; 
localparam S_WAIT             = 12; 
localparam S_WR_REG_ADDR1     = 13; 
localparam S_RD_REG_ADDR1     = 14; 
localparam S_RD_ACK           = 15; 
reg start;
reg stop;
reg read;
reg write;
reg ack_in;
reg[7:0] txr;
wire[7:0] rxr;
wire i2c_busy;
wire i2c_al;
wire done;
wire irxack;
reg[3:0] state, next_state;
assign i2c_read_req_ack = (state == S_RD_ACK);
assign i2c_write_req_ack = (state == S_WR_ACK);
always@(posedge clk or posedge rst)
begin
	if(rst)
		state <= S_IDLE;
	else
		state <= next_state;    
end
always@(*)
begin
	case(state)
		S_IDLE:
			//Waiting for read and write requests
			if(i2c_write_req)
				next_state <= S_WR_DEV_ADDR;
			else if(i2c_read_req)
				next_state <= S_RD_DEV_ADDR0;
			else
				next_state <= S_IDLE;
		//Write I2C device address
		S_WR_DEV_ADDR:
			if(done && irxack)
				next_state <= S_WR_ERR_NACK;
			else if(done)
				next_state <= S_WR_REG_ADDR;
			else
				next_state <= S_WR_DEV_ADDR;
		//Write the address of the I2C register
		S_WR_REG_ADDR:
			if(done)
				//If it is the 8bit register address, it enters the write data state
				next_state <= i2c_addr_2byte ? S_WR_REG_ADDR1 : S_WR_DATA;
			else
				next_state <= S_WR_REG_ADDR;
		S_WR_REG_ADDR1:
			if(done)
				next_state <= S_WR_DATA;
			else
				next_state <= S_WR_REG_ADDR1; 
		//Write data		
		S_WR_DATA:
			if(done)
				next_state <= S_WR_STOP;
			else
				next_state <= S_WR_DATA;
		S_WR_ERR_NACK:
			next_state <= S_WR_STOP;
		S_RD_ACK,S_WR_ACK:
			next_state <= S_WAIT;
		S_WAIT:
			next_state <= S_IDLE;
		S_RD_DEV_ADDR0:
			if(done && irxack)
				next_state <= S_WR_ERR_NACK;
			else if(done)
				next_state <= S_RD_REG_ADDR;
			else
				next_state <= S_RD_DEV_ADDR0;
		S_RD_REG_ADDR:
			if(done)
				next_state <= i2c_addr_2byte ? S_RD_REG_ADDR1 : S_RD_DEV_ADDR1;
			else
				next_state <= S_RD_REG_ADDR;
		S_RD_REG_ADDR1:
			if(done)
				next_state <= S_RD_DEV_ADDR1;
			else
				next_state <= S_RD_REG_ADDR1;               
		S_RD_DEV_ADDR1:
			if(done)
				next_state <= S_RD_DATA;
			else
				next_state <= S_RD_DEV_ADDR1;   
		S_RD_DATA:
			if(done)
				next_state <= S_RD_STOP;
			else
				next_state <= S_RD_DATA;
		S_RD_STOP:
			if(done)
				next_state <= S_RD_ACK;
			else
				next_state <= S_RD_STOP;
		S_WR_STOP:
			if(done)
				next_state <= S_WR_ACK;
			else
				next_state <= S_WR_STOP;                
		default:
			next_state <= S_IDLE;
	endcase
end

always@(posedge clk or posedge rst)
begin
	if(rst)
		error <= 1'b0;
	else if(state == S_IDLE)
		error <= 1'b0;
	else if(state == S_WR_ERR_NACK)
		error <= 1'b1;
end

always@(posedge clk or posedge rst)
begin
	if(rst)
		start <= 1'b0;
	else if(done)
		start <= 1'b0;
	else if(state == S_WR_DEV_ADDR || state == S_RD_DEV_ADDR0 || state == S_RD_DEV_ADDR1)
		start <= 1'b1;
end
always@(posedge clk or posedge rst)
begin
	if(rst)
		stop <= 1'b0;
	else if(done)
		stop <= 1'b0;
	else if(state == S_WR_STOP || state == S_RD_STOP)
		stop <= 1'b1;
end
always@(posedge clk or posedge rst)
begin
	if(rst)
		ack_in <= 1'b0;
	else 
		ack_in <= 1'b1;
end
always@(posedge clk or posedge rst)
begin
	if(rst)
		write <= 1'b0;
	else if(done)
		write <= 1'b0;
	else if(state == S_WR_DEV_ADDR || state == S_WR_REG_ADDR || state == S_WR_REG_ADDR1|| state == S_WR_DATA || state == S_RD_DEV_ADDR0 || state == S_RD_DEV_ADDR1 || state == S_RD_REG_ADDR || state == S_RD_REG_ADDR1)
		write <= 1'b1;
end
always@(posedge clk or posedge rst)
begin
	if(rst)
		read <= 1'b0;
	else if(done)
		read <= 1'b0;
	else if(state == S_RD_DATA)
		read <= 1'b1;
end

always@(posedge clk or posedge rst)
begin
	if(rst)
		i2c_read_data <= 8'h00;
	else if(state == S_RD_DATA && done)
		i2c_read_data <= rxr;
end

always@(posedge clk or posedge rst)
begin
	if(rst)
		txr <= 8'd0;
	else 
		case(state)
			S_WR_DEV_ADDR,S_RD_DEV_ADDR0:
				txr <= {i2c_slave_dev_addr[7:1],1'b0};//The LSB is 0 was express write operation
			S_RD_DEV_ADDR1:
				txr <= {i2c_slave_dev_addr[7:1],1'b1};//The MSB is 1 was express write operation
			S_WR_REG_ADDR,S_RD_REG_ADDR:
				txr <= (i2c_addr_2byte == 1'b1) ? i2c_slave_reg_addr[15:8] : i2c_slave_reg_addr[7:0];//General firstly send the low byte
			S_WR_REG_ADDR1,S_RD_REG_ADDR1:
				txr <= i2c_slave_reg_addr[7:0];             
			S_WR_DATA:
				txr <= i2c_write_data;
			default:
				txr <= 8'hff;
		endcase
end
i2c_master_byte_ctrl byte_controller 
(
	.clk      ( clk          ),
	.rst      ( rst          ),
	.nReset   ( 1'b1         ),
	.ena      ( 1'b1         ),
	.clk_cnt  ( clk_div_cnt  ),
	.start    ( start        ),
	.stop     ( stop         ),
	.read     ( read         ),
	.write    ( write        ),
	.ack_in   ( ack_in       ),
	.din      ( txr          ),
	.cmd_ack  ( done         ),
	.ack_out  ( irxack       ),
	.dout     ( rxr          ),
	.i2c_busy ( i2c_busy     ),
	.i2c_al   ( i2c_al       ),
	.scl_i    ( scl_pad_i    ),
	.scl_o    ( scl_pad_o    ),
	.scl_oen  ( scl_padoen_o ),
	.sda_i    ( sda_pad_i    ),
	.sda_o    ( sda_pad_o    ),
	.sda_oen  ( sda_padoen_o )
);
endmodule 

module  ax_debounce 
(
    input       clk, 
    input       rst, 
    input       button_in,
    output reg  button_posedge,
    output reg  button_negedge,
    output reg  button_out
);
parameter N = 32 ;           // debounce timer bitwidth
parameter FREQ = 50;         //model clock :Mhz
parameter MAX_TIME = 20;     //ms
localparam TIMER_MAX_VAL =   MAX_TIME * 1000 * FREQ;

reg  [N-1 : 0]  q_reg;      // timing regs
reg  [N-1 : 0]  q_next;
reg DFF1, DFF2;             // input flip-flops
wire q_add;                 // control flags
wire q_reset;
reg button_out_d0;


assign q_reset = (DFF1  ^ DFF2);          // xor input flip flops to look for level chage to reset counter
assign q_add = ~(q_reg == TIMER_MAX_VAL); // add to counter when q_reg msb is equal to 0
    
always @ ( q_reset, q_add, q_reg)
begin
    case( {q_reset , q_add})
        2'b00 :
                q_next <= q_reg;
        2'b01 :
                q_next <= q_reg + 1;
        default :
                q_next <= { N {1'b0} };
    endcase     
end

always @ ( posedge clk or posedge rst)
begin
    if(rst == 1'b1)
    begin
        DFF1 <= 1'b0;
        DFF2 <= 1'b0;
        q_reg <= { N {1'b0} };
    end
    else
    begin
        DFF1 <= button_in;
        DFF2 <= DFF1;
        q_reg <= q_next;
    end
end

always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		button_out <= 1'b1;
    else if(q_reg == TIMER_MAX_VAL)
        button_out <= DFF2;
    else
        button_out <= button_out;
end

always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		button_out_d0 <= 1'b1;
		button_posedge <= 1'b0;
		button_negedge <= 1'b0;
	end
	else
	begin
		button_out_d0 <= button_out;
		button_posedge <= ~button_out_d0 & button_out;
		button_negedge <= button_out_d0 & ~button_out;
	end	
end
endmodule

module i2c_master_byte_ctrl (
	                          clk, rst, nReset, ena,
									  clk_cnt, start, stop,
									  read, write, ack_in, din,
	                          cmd_ack, ack_out, dout, 
									  i2c_busy, i2c_al, scl_i, 
									  scl_o, scl_oen, sda_i,
									  sda_o, sda_oen 
									 );

	//
	// inputs & outputs
	//
	input clk;     // master clock
	input rst;     // synchronous active high reset
	input nReset;  // asynchronous active low reset  always "1"
	input ena;     // core enable signal  always "1"

	input [15:0] clk_cnt; // 4x SCL

	// control inputs
	input       start;
	input       stop;
	input       read;
	input       write;
	input       ack_in;
	input [7:0] din;

	// status outputs
	output       cmd_ack;
	reg cmd_ack;
	output       ack_out;
	reg ack_out;
	output       i2c_busy;
	output       i2c_al;
	output [7:0] dout;

	// I2C signals
	input  scl_i;
	output scl_o;
	output scl_oen;
	input  sda_i;
	output sda_o;
	output sda_oen;


	//
	// Variable declarations
	//

	// statemachine
	parameter [4:0] ST_IDLE  = 5'b0_0000;
	parameter [4:0] ST_START = 5'b0_0001;
	parameter [4:0] ST_READ  = 5'b0_0010;
	parameter [4:0] ST_WRITE = 5'b0_0100;
	parameter [4:0] ST_ACK   = 5'b0_1000;
	parameter [4:0] ST_STOP  = 5'b1_0000;

	// signals for bit_controller
	reg  [3:0] core_cmd;
	reg        core_txd;
	wire       core_ack, core_rxd;

	// signals for shift register
	reg [7:0] sr; //8bit shift register
	reg       shift, ld;

	// signals for state machine
	wire       go;
	reg  [2:0] dcnt;
	wire       cnt_done;

	//
	// Module body
	//

	// hookup bit_controller
	i2c_master_bit_ctrl bit_controller (
		.clk     ( clk      ),
		.rst     ( rst      ),
		.nReset  ( nReset   ),
		.ena     ( ena      ),
		.clk_cnt ( clk_cnt  ),
		.cmd     ( core_cmd ),
		.cmd_ack ( core_ack ),
		.busy    ( i2c_busy ),
		.al      ( i2c_al   ),
		.din     ( core_txd ),
		.dout    ( core_rxd ),
		.scl_i   ( scl_i    ),
		.scl_o   ( scl_o    ),
		.scl_oen ( scl_oen  ),
		.sda_i   ( sda_i    ),
		.sda_o   ( sda_o    ),
		.sda_oen ( sda_oen  )
	);

	// generate go-signal
	assign go = (read | write | stop) & ~cmd_ack;

	// assign dout output to shift-register
	assign dout = sr;

	// generate shift register
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    sr <= #1 8'h0;
	  else if (rst)
	    sr <= #1 8'h0;
	  else if (ld)    //if have any command will driver "ld"
	    sr <= #1 din; //wrtie data in
	  else if (shift)
	    sr <= #1 {sr[6:0], core_rxd};

	// generate counter
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    dcnt <= #1 3'h0;
	  else if (rst)
	    dcnt <= #1 3'h0;
	  else if (ld)
	    dcnt <= #1 3'h7;
	  else if (shift)
	    dcnt <= #1 dcnt - 3'h1;

	assign cnt_done = ~(|dcnt);//Abbreviated operation,dcnt=0 and cnt_done =1

	//
	// state machine
	//
	reg [4:0] c_state; // synopsys enum_state

	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    begin
	        core_cmd <= #1 `I2C_CMD_NOP;
	        core_txd <= #1 1'b0;
	        shift    <= #1 1'b0;
	        ld       <= #1 1'b0;
	        cmd_ack  <= #1 1'b0;
	        c_state  <= #1 ST_IDLE;
	        ack_out  <= #1 1'b0;
	    end
	  else if (rst | i2c_al)
	   begin
	       core_cmd <= #1 `I2C_CMD_NOP;
	       core_txd <= #1 1'b0;
	       shift    <= #1 1'b0;
	       ld       <= #1 1'b0;
	       cmd_ack  <= #1 1'b0;
	       c_state  <= #1 ST_IDLE;
	       ack_out  <= #1 1'b0;
	   end
	else
	  begin
	      // initially reset all signals
	      core_txd <= #1 sr[7];
	      shift    <= #1 1'b0;
	      ld       <= #1 1'b0;
	      cmd_ack  <= #1 1'b0;

	      case (c_state) // synopsys full_case parallel_case
	        ST_IDLE:
	          if (go)
	            begin
	                if (start)
	                  begin
	                      c_state  <= #1 ST_START;
	                      core_cmd <= #1 `I2C_CMD_START;
	                  end
	                else if (read)
	                  begin
	                      c_state  <= #1 ST_READ;
	                      core_cmd <= #1 `I2C_CMD_READ;
	                  end
	                else if (write)
	                  begin
	                      c_state  <= #1 ST_WRITE;
	                      core_cmd <= #1 `I2C_CMD_WRITE;
	                  end
	                else // stop
	                  begin
	                      c_state  <= #1 ST_STOP;
	                      core_cmd <= #1 `I2C_CMD_STOP;
	                  end

	                ld <= #1 1'b1;
	            end

	        ST_START:
	          if (core_ack)
	            begin
	                if (read)
	                  begin
	                      c_state  <= #1 ST_READ;
	                      core_cmd <= #1 `I2C_CMD_READ;
	                  end
	                else
	                  begin
	                      c_state  <= #1 ST_WRITE;
	                      core_cmd <= #1 `I2C_CMD_WRITE;
	                  end

	                ld <= #1 1'b1;
	            end

	        ST_WRITE:
	          if (core_ack)
	            if (cnt_done)
	              begin
	                  c_state  <= #1 ST_ACK;
	                  core_cmd <= #1 `I2C_CMD_READ;
	              end
	            else
	              begin
	                  c_state  <= #1 ST_WRITE;       // stay in same state
	                  core_cmd <= #1 `I2C_CMD_WRITE; // write next bit
	                  shift    <= #1 1'b1;
	              end

	        ST_READ:
	          if (core_ack)
	            begin
	                if (cnt_done)
	                  begin
	                      c_state  <= #1 ST_ACK;
	                      core_cmd <= #1 `I2C_CMD_WRITE;
	                  end
	                else
	                  begin
	                      c_state  <= #1 ST_READ;       // stay in same state
	                      core_cmd <= #1 `I2C_CMD_READ; // read next bit
	                  end

	                shift    <= #1 1'b1;
	                core_txd <= #1 ack_in;              //it's noack bit 
	            end

	        ST_ACK:
	          if (core_ack)
	            begin
	               if (stop)
	                 begin
	                     c_state  <= #1 ST_STOP;
	                     core_cmd <= #1 `I2C_CMD_STOP;
	                 end
	               else
	                 begin
	                     c_state  <= #1 ST_IDLE;
	                     core_cmd <= #1 `I2C_CMD_NOP;

	                     // generate command acknowledge signal
	                     cmd_ack  <= #1 1'b1;
	                 end

	                 // assign ack_out output to bit_controller_rxd (contains last received bit)
	                 ack_out <= #1 core_rxd;

	                 core_txd <= #1 1'b1;
	             end
	           else
	             core_txd <= #1 ack_in;  

	        ST_STOP:
	          if (core_ack)
	            begin
	                c_state  <= #1 ST_IDLE;
	                core_cmd <= #1 `I2C_CMD_NOP;

	                // generate command acknowledge signal
	                cmd_ack  <= #1 1'b1;
	            end

	      endcase
	  end
endmodule

// Bit controller section

//
// Translate simple commands into SCL/SDA transitions
// Each command has 5 states, A/B/C/D/idle
//
// start:	SCL	~~~~~~~~~~\____
//	SDA	~~~~~~~~\______
//		 x | A | B | C | D | i
//
// repstart	SCL	____/~~~~\___
//	SDA	__/~~~\______
//		 x | A | B | C | D | i
//
// stop	SCL	____/~~~~~~~~
//	SDA	==\____/~~~~~
//		 x | A | B | C | D | i
//
//- write	SCL	____/~~~~\____
//	SDA	==X=========X=
//		 x | A | B | C | D | i
//
//- read	SCL	____/~~~~\____
//	SDA	XXXX=====XXXX
//		 x | A | B | C | D | i
//

// Timing:     Normal mode      Fast mode
///
// Fscl        100KHz           400KHz
// Th_scl      4.0us            0.6us   High period of SCL
// Tl_scl      4.7us            1.3us   Low period of SCL
// Tsu:sta     4.7us            0.6us   setup time for a repeated start condition
// Tsu:sto     4.0us            0.6us   setup time for a stop conditon
// Tbuf        4.7us            1.3us   Bus free time between a stop and start condition
//

module i2c_master_bit_ctrl (
    input             clk,      // system clock
    input             rst,      // synchronous active high reset
    input             nReset,   // asynchronous active low reset
    input             ena,      // core enable signal

    input      [15:0] clk_cnt,  // clock prescale value

    input      [ 3:0] cmd,      // command (from byte controller)
    output reg        cmd_ack,  // command complete acknowledge
    output reg        busy,     // i2c bus busy
    output reg        al,       // i2c bus arbitration lost

    input             din,
    output reg        dout,

    input             scl_i,    // i2c clock line input
    output            scl_o,    // i2c clock line output
    output reg        scl_oen,  // i2c clock line output enable (active low)
    input             sda_i,    // i2c data line input
    output            sda_o,    // i2c data line output
    output reg        sda_oen   // i2c data line output enable (active low)
);


    //
    // variable declarations
    //

    reg [1:0]  cSCL, cSDA;       // capture SCL and SDA
    reg [2:0]  fSCL, fSDA;       // SCL and SDA filter inputs
    reg        sSCL, sSDA;      // filtered and synchronized SCL and SDA inputs
    reg        dSCL, dSDA;      // delayed versions of sSCL and sSDA
    reg        dscl_oen;        // delayed scl_oen
    reg        sda_chk;         // check SDA output (Multi-master arbitration)
    reg        clk_en;          // clock generation signals
    reg        slave_wait;      // slave inserts wait states
    reg [15:0] cnt;             // clock divider counter (synthesis)
    reg [13:0] filter_cnt;      // clock divider for filter


    // state machine variable
    reg [17:0] c_state; // synopsys enum_state

    //
    // module body
    //

    // whenever the slave is not ready it can delay the cycle by pulling SCL low
    // delay scl_oen
    always @(posedge clk)
      dscl_oen <= #1 scl_oen;

    // slave_wait is asserted when master wants to drive SCL high, but the slave pulls it low
    // slave_wait remains asserted until the slave releases SCL
    always @(posedge clk or negedge nReset)
      if (!nReset) slave_wait <= 1'b0;
      else         slave_wait <= (scl_oen & ~dscl_oen & ~sSCL) | (slave_wait & ~sSCL);

    // master drives SCL high, but another master pulls it low
    // master start counting down its low cycle now (clock synchronization)
    wire scl_sync   = dSCL & ~sSCL & scl_oen;


    // generate clk enable signal
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          cnt    <= #1 16'h0;
          clk_en <= #1 1'b1;
      end
      else if (rst || ~|cnt || !ena || scl_sync)
      begin
          cnt    <= #1 clk_cnt;
          clk_en <= #1 1'b1;
      end
      else if (slave_wait)
      begin
          cnt    <= #1 cnt;
          clk_en <= #1 1'b0;    
      end
      else
      begin
          cnt    <= #1 cnt - 16'h1;
          clk_en <= #1 1'b0;
      end


    // generate bus status controller

    // capture SDA and SCL
    // reduce metastability risk
    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          cSCL <= #1 2'b00;
          cSDA <= #1 2'b00;
      end
      else if (rst)
      begin
          cSCL <= #1 2'b00;
          cSDA <= #1 2'b00;
      end
      else
      begin
          cSCL <= {cSCL[0],scl_i};
          cSDA <= {cSDA[0],sda_i};
      end


    // filter SCL and SDA signals; (attempt to) remove glitches
    always @(posedge clk or negedge nReset)
      if      (!nReset     ) filter_cnt <= 14'h0;
      else if (rst || !ena ) filter_cnt <= 14'h0;
      else if (~|filter_cnt) filter_cnt <= clk_cnt >> 2; //16x I2C bus frequency
      else                   filter_cnt <= filter_cnt -1;


    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          fSCL <= 3'b111;
          fSDA <= 3'b111;
      end
      else if (rst)
      begin
          fSCL <= 3'b111;
          fSDA <= 3'b111;
      end
      else if (~|filter_cnt)
      begin
          fSCL <= {fSCL[1:0],cSCL[1]};
          fSDA <= {fSDA[1:0],cSDA[1]};
      end


    // generate filtered SCL and SDA signals
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          sSCL <= #1 1'b1;
          sSDA <= #1 1'b1;

          dSCL <= #1 1'b1;
          dSDA <= #1 1'b1;
      end
      else if (rst)
      begin
          sSCL <= #1 1'b1;
          sSDA <= #1 1'b1;

          dSCL <= #1 1'b1;
          dSDA <= #1 1'b1;
      end
      else
      begin
          sSCL <= #1 &fSCL[2:1] | &fSCL[1:0] | (fSCL[2] & fSCL[0]);
          sSDA <= #1 &fSDA[2:1] | &fSDA[1:0] | (fSDA[2] & fSDA[0]);

          dSCL <= #1 sSCL;
          dSDA <= #1 sSDA;
      end

    // detect start condition => detect falling edge on SDA while SCL is high
    // detect stop condition => detect rising edge on SDA while SCL is high
    reg sta_condition;
    reg sto_condition;
    always @(posedge clk or negedge nReset)
      if (~nReset)
      begin
          sta_condition <= #1 1'b0;
          sto_condition <= #1 1'b0;
      end
      else if (rst)
      begin
          sta_condition <= #1 1'b0;
          sto_condition <= #1 1'b0;
      end
      else
      begin
          sta_condition <= #1 ~sSDA &  dSDA & sSCL;
          sto_condition <= #1  sSDA & ~dSDA & sSCL;
      end


    // generate i2c bus busy signal
    always @(posedge clk or negedge nReset)
      if      (!nReset) busy <= #1 1'b0;
      else if (rst    ) busy <= #1 1'b0;
      else              busy <= #1 (sta_condition | busy) & ~sto_condition;


    // generate arbitration lost signal
    // aribitration lost when:
    // 1) master drives SDA high, but the i2c bus is low
    // 2) stop detected while not requested
    reg cmd_stop;
    always @(posedge clk or negedge nReset)
      if (~nReset)
          cmd_stop <= #1 1'b0;
      else if (rst)
          cmd_stop <= #1 1'b0;
      else if (clk_en)
          cmd_stop <= #1 cmd == `I2C_CMD_STOP;

    always @(posedge clk or negedge nReset)
      if (~nReset)
          al <= #1 1'b0;
      else if (rst)
          al <= #1 1'b0;
      else
          al <= #1 (sda_chk & ~sSDA & sda_oen) | (|c_state & sto_condition & ~cmd_stop);


    // generate dout signal (store SDA on rising edge of SCL)
    always @(posedge clk)
      if (sSCL & ~dSCL)
		dout <= #1 sSDA;


    // generate statemachine

    // nxt_state decoder
    parameter [17:0] idle    = 18'b0_0000_0000_0000_0000;
    parameter [17:0] start_a = 18'b0_0000_0000_0000_0001;
    parameter [17:0] start_b = 18'b0_0000_0000_0000_0010;
    parameter [17:0] start_c = 18'b0_0000_0000_0000_0100;
    parameter [17:0] start_d = 18'b0_0000_0000_0000_1000;
    parameter [17:0] start_e = 18'b0_0000_0000_0001_0000;
    parameter [17:0] stop_a  = 18'b0_0000_0000_0010_0000;
    parameter [17:0] stop_b  = 18'b0_0000_0000_0100_0000;
    parameter [17:0] stop_c  = 18'b0_0000_0000_1000_0000;
    parameter [17:0] stop_d  = 18'b0_0000_0001_0000_0000;
    parameter [17:0] rd_a    = 18'b0_0000_0010_0000_0000;
    parameter [17:0] rd_b    = 18'b0_0000_0100_0000_0000;
    parameter [17:0] rd_c    = 18'b0_0000_1000_0000_0000;
    parameter [17:0] rd_d    = 18'b0_0001_0000_0000_0000;
    parameter [17:0] wr_a    = 18'b0_0010_0000_0000_0000;
    parameter [17:0] wr_b    = 18'b0_0100_0000_0000_0000;
    parameter [17:0] wr_c    = 18'b0_1000_0000_0000_0000;
    parameter [17:0] wr_d    = 18'b1_0000_0000_0000_0000;

    always @(posedge clk or negedge nReset)
      if (!nReset)
      begin
          c_state <= #1 idle;
          cmd_ack <= #1 1'b0;
          scl_oen <= #1 1'b1;
          sda_oen <= #1 1'b1;
          sda_chk <= #1 1'b0;
      end
      else if (rst | al)
      begin
          c_state <= #1 idle;
          cmd_ack <= #1 1'b0;
          scl_oen <= #1 1'b1;
          sda_oen <= #1 1'b1;
          sda_chk <= #1 1'b0;
      end
      else
      begin
          cmd_ack   <= #1 1'b0; // default no command acknowledge + assert cmd_ack only 1clk cycle

          if (clk_en)
              case (c_state) // synopsys full_case parallel_case
                    // idle state
                    idle:
                    begin
                        case (cmd) // synopsys full_case parallel_case
                             `I2C_CMD_START: c_state <= #1 start_a;
                             `I2C_CMD_STOP:  c_state <= #1 stop_a;
                             `I2C_CMD_WRITE: c_state <= #1 wr_a;
                             `I2C_CMD_READ:  c_state <= #1 rd_a;
                             default:        c_state <= #1 idle;
                        endcase

                        scl_oen <= #1 scl_oen; // keep SCL in same state
                        sda_oen <= #1 sda_oen; // keep SDA in same state
                        sda_chk <= #1 1'b0;    // don't check SDA output
                    end

                    // start
                    start_a:
                    begin
                        c_state <= #1 start_b;
                        scl_oen <= #1 scl_oen; // keep SCL in same state
                        sda_oen <= #1 1'b1;    // set SDA high
                        sda_chk <= #1 1'b0;    // don't check SDA output
                    end

                    start_b:
                    begin
                        c_state <= #1 start_c;
                        scl_oen <= #1 1'b1; // set SCL high
                        sda_oen <= #1 1'b1; // keep SDA high
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    start_c:
                    begin
                        c_state <= #1 start_d;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 1'b0; // set SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    start_d:
                    begin
                        c_state <= #1 start_e;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 1'b0; // keep SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    start_e:
                    begin
                        c_state <= #1 idle;
                        cmd_ack <= #1 1'b1;
                        scl_oen <= #1 1'b0; // set SCL low
                        sda_oen <= #1 1'b0; // keep SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    // stop
                    stop_a:
                    begin
                        c_state <= #1 stop_b;
                        scl_oen <= #1 1'b0; // keep SCL low
                        sda_oen <= #1 1'b0; // set SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    stop_b:
                    begin
                        c_state <= #1 stop_c;
                        scl_oen <= #1 1'b1; // set SCL high
                        sda_oen <= #1 1'b0; // keep SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    stop_c:
                    begin
                        c_state <= #1 stop_d;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 1'b0; // keep SDA low
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    stop_d:
                    begin
                        c_state <= #1 idle;
                        cmd_ack <= #1 1'b1;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 1'b1; // set SDA high
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    // read
                    rd_a:
                    begin
                        c_state <= #1 rd_b;
                        scl_oen <= #1 1'b0; // keep SCL low
                        sda_oen <= #1 1'b1; // tri-state SDA
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    rd_b:
                    begin
                        c_state <= #1 rd_c;
                        scl_oen <= #1 1'b1; // set SCL high
                        sda_oen <= #1 1'b1; // keep SDA tri-stated
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    rd_c:
                    begin
                        c_state <= #1 rd_d;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 1'b1; // keep SDA tri-stated
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    rd_d:
                    begin
                        c_state <= #1 idle;
                        cmd_ack <= #1 1'b1;
                        scl_oen <= #1 1'b0; // set SCL low
                        sda_oen <= #1 1'b1; // keep SDA tri-stated
                        sda_chk <= #1 1'b0; // don't check SDA output
                    end

                    // write
                    wr_a:
                    begin
                        c_state <= #1 wr_b;
                        scl_oen <= #1 1'b0; // keep SCL low
                        sda_oen <= #1 din;  // set SDA
                        sda_chk <= #1 1'b0; // don't check SDA output (SCL low)
                    end

                    wr_b:
                    begin
                        c_state <= #1 wr_c;
                        scl_oen <= #1 1'b1; // set SCL high
                        sda_oen <= #1 din;  // keep SDA
                        sda_chk <= #1 1'b0; // don't check SDA output yet
                                            // allow some time for SDA and SCL to settle
                    end

                    wr_c:
                    begin
                        c_state <= #1 wr_d;
                        scl_oen <= #1 1'b1; // keep SCL high
                        sda_oen <= #1 din;
                        sda_chk <= #1 1'b1; // check SDA output
                    end

                    wr_d:
                    begin
                        c_state <= #1 idle;
                        cmd_ack <= #1 1'b1;
                        scl_oen <= #1 1'b0; // set SCL low
                        sda_oen <= #1 din;
                        sda_chk <= #1 1'b0; // don't check SDA output (SCL low)
                    end

              endcase
      end


    // assign scl and sda output (always gnd)
    assign scl_o = 1'b0;
    assign sda_o = 1'b0;

endmodule
