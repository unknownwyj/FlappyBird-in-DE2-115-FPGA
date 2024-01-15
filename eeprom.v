
`include "param.v"

module eeprom_ctrl (
    input               clk         ,
    input               rst_n       ,
    //按键使能
    input               rd_en       ,
    //串口信号
    input       [7:0]   din         ,
    input               din_vld     ,
    output      [7:0]   dout        ,
    output              dout_vld    ,
    input               ready    ,
    //IIC接口信号
    output              req         ,
    output      [3:0]   cmd         ,
    output      [7:0]   wr_data     ,//向EEPROM写数据
    input       [7:0]   rd_data     ,//从EEPROM读数据
    input               done        
);

//状态机参数
    localparam      IDLE    = 6'b00_0001    ,
                    WR_REQ  = 6'b00_0010    ,//写传输 发送请求、命令、数据
                    WAIT_WR = 6'b00_0100    ,//等待一个字节传完
                    RD_REQ  = 6'b00_1000    ,//读传输 发送请求、命令、数据
                    WAIT_RD = 6'b01_0000    ,//等待一个自己传完
                    DONE    = 6'b10_0000    ;//一次读或写完成
//信号定义
    reg [5:0]       state_c;//现态
    reg [5:0]       state_n;//次态

    reg [3:0]       cnt_byte;
    wire            add_cnt_byte;  
    wire            end_cnt_byte;  

    reg             tx_req          ;//请求
    reg     [3:0]   tx_cmd          ;
    reg     [7:0]   tx_data         ;

    reg     [8:0]   wr_addr         ;//写eeprom地址
    reg     [8:0]   rd_addr         ;//读eeprom地址

    reg     [7:0]   dout_r          ;
    reg             dout_vld_r      ;

    wire            wfifo_rd        ;
    wire            wfifo_wr        ;
    wire            wfifo_empty     ;
    wire            wfifo_full      ;
    wire    [7:0]   wfifo_qout      ;
    wire    [5:0]   wfifo_usedw     ;

    wire            rfifo_rd        ;
    wire            rfifo_wr        ;
    wire            rfifo_empty     ;
    wire            rfifo_full      ;
    wire    [7:0]   rfifo_qout      ;
    wire    [5:0]   rfifo_usedw     ;

    wire            idle2wr_req     ;
    wire            wr_req2wait_wr  ;
    wire            wait_wr2wr_req  ;
    wire            wait_wr2done    ;
    wire            idle2rd_req     ;
    wire            rd_req2wait_rd  ;
    wire            wait_rd2rd_req  ;
    wire            wait_rd2done    ;
    wire            done2idle       ;
//状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_c <= IDLE;
        end
        else
            state_c <= state_n;
    end

    always @(*) begin 
            case(state_c)  
                IDLE :begin
                    if(idle2wr_req)
                        state_n = WR_REQ ;
                    else if(idle2rd_req)
                        state_n = RD_REQ ;
                    else 
                        state_n = state_c ;
                end
                WR_REQ :begin
                    if(wr_req2wait_wr)
                        state_n = WAIT_WR ;
                    else 
                        state_n = state_c ;
                end
                WAIT_WR :begin
                    if(wait_wr2wr_req)
                        state_n = WR_REQ ;
                    else if(wait_wr2done)
                        state_n = DONE ;
                    else 
                        state_n = state_c ;
                end
                RD_REQ :begin
                    if(rd_req2wait_rd)
                        state_n = WAIT_RD ;
                    else 
                        state_n = state_c ;
                end
                WAIT_RD :begin
                    if(wait_rd2rd_req)
                        state_n = RD_REQ ;
                    else if(wait_rd2done)
                        state_n = DONE ;
                    else 
                        state_n = state_c ;
                end
                DONE :begin
                    if(done2idle)
                        state_n = IDLE ;
                    else 
                        state_n = state_c ;
                end
                default : state_n = IDLE ;
            endcase
        end

    assign  idle2wr_req    = state_c == IDLE     && (wfifo_usedw > 0);
    assign  wr_req2wait_wr = state_c == WR_REQ   && (1);
    assign  wait_wr2wr_req = state_c == WAIT_WR  && (done & cnt_byte < `WR_BYTE-1);
    assign  wait_wr2done   = state_c == WAIT_WR  && (end_cnt_byte);
    assign  idle2rd_req    = state_c == IDLE     && (rd_en);
    assign  rd_req2wait_rd = state_c == RD_REQ   && (1);
    assign  wait_rd2rd_req = state_c == WAIT_RD  && (done & cnt_byte < `RD_BYTE-1);
    assign  wait_rd2done   = state_c == WAIT_RD  && (end_cnt_byte);
    assign  done2idle      = state_c == DONE     && (1);

//cnt_byte
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_byte <= 0;
        end
        else if(add_cnt_byte) begin
            if (end_cnt_byte) begin
                cnt_byte <= 0;
            end
            else
                cnt_byte <= cnt_byte+1;
        end
    end
    
    assign add_cnt_byte = ((state_c == WAIT_RD || state_c == WAIT_WR) && done);
    assign end_cnt_byte = add_cnt_byte && cnt_byte == ((state_c == WAIT_WR) ? `WR_BYTE : `RD_BYTE) -1;

//输出

    always  @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            TX(1'b0,4'd0,8'd0);
        end
        else if(state_c==WR_REQ)begin
            case(cnt_byte)
                0           :TX(1'b1,{`CMD_START | `CMD_WRITE},{`I2C_ADR,wr_addr[8],`WR_BIT});//发起始位、写控制字
                1           :TX(1'b1,`CMD_WRITE,wr_addr[7:0]);   //发 写地址
                `WR_BYTE-1  :TX(1'b1,{`CMD_WRITE | `CMD_STOP},wfifo_qout);  //最后一个字节时 发数据、停止位
                default     :TX(1'b1,`CMD_WRITE,wfifo_qout);    //中间发数据（如果有）
            endcase 
        end
        else if(state_c==RD_REQ)begin
            case(cnt_byte)
                0           :TX(1'b1,{`CMD_START | `CMD_WRITE},{`I2C_ADR,rd_addr[8],`WR_BIT});//发起始位、写控制字
                1           :TX(1'b1,`CMD_WRITE,rd_addr[7:0]);   //发 读地址
                2           :TX(1'b1,{`CMD_START | `CMD_WRITE},{`I2C_ADR,rd_addr[8],`RD_BIT});//发起始位、读控制字
                `RD_BYTE-1  :TX(1'b1,{`CMD_READ | `CMD_STOP},0);  //最后一个字节时 读数据、发停止位
                default     :TX(1'b1,`CMD_READ,0);    //中间读数据（如果有）
            endcase 
        end
        else begin 
             TX(1'b0,tx_cmd,tx_data);
        end 
    end


//用task发送请求、命令、数据（地址+数据）
    task TX;   
        input                   req     ;
        input       [3:0]       command ;
        input       [7:0]       data    ;
        begin 
            tx_req  = req;
            tx_cmd  = command;
            tx_data = data;
        end 
    endtask 

//wr_addr   rd_addr
    always  @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            wr_addr <= 0;
        end
        else if(wait_wr2done)begin
            wr_addr <= wr_addr + `WR_BYTE-2;
        end
    end
    
    always  @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            rd_addr <= 0;
        end
        else if(wait_rd2done)begin
            rd_addr <= rd_addr + `RD_BYTE - 3;
        end
    end


//dout_r dout_vld_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout_r <= 0;
            dout_vld_r <= 0;
        end
        else begin
            dout_r <= rfifo_qout;
            dout_vld_r <= rfifo_rd;
        end
    end

//输出

    assign req     = tx_req ; 
    assign cmd     = tx_cmd ; 
    assign wr_data = tx_data; 

    assign dout = dout_r;
    assign dout_vld = dout_vld_r;
//fifo例化

    wrfifo	u_wrfifo (
    	.aclr   (~rst_n     ),
    	.clock  (clk        ),
    	.data   (din        ),
    	.rdreq  (wfifo_rd   ),
    	.wrreq  (wfifo_wr   ),
    	.empty  (wfifo_empty),
    	.full   (wfifo_full ),
    	.q      (wfifo_qout ),
    	.usedw  (wfifo_usedw)
    );

    assign wfifo_rd = state_c==WAIT_WR && done && cnt_byte > 1;//读使能（脉冲信号）---IIC接口传输完控制字节，字地址，cnt_byte==2时读出数据
    assign wfifo_wr = ~wfifo_full & din_vld;//写使能（脉冲信号）---fifo非满且uart_rx模块数据有效

    rdfifo	u_rdfifo (
	    .aclr   (~rst_n     ),
	    .clock  (clk        ),
	    .data   (rd_data    ),
	    .rdreq  (rfifo_rd   ),
	    .wrreq  (rfifo_wr   ),
	    .empty  (rfifo_empty),
	    .full   (rfifo_full ),
	    .q      (rfifo_qout ),
	    .usedw  (rfifo_usedw)
	);

    assign rfifo_wr = ~rfifo_full && state_c == WAIT_RD && done && cnt_byte > 2;
    assign rfifo_rd = ~rfifo_empty && ready;
endmodule
module i2c_master (
    input               clk         ,
    input               rst_n       ,
    //控制信号
    input               req         ,//请求
    input       [3:0]   cmd         ,//命令
    input       [7:0]   din         ,//写数据
    //IIC信号
    output              i2c_scl     ,//SCL---200kHz
    input               i2c_sda_i   ,//SDA---输入
    output              i2c_sda_o   ,//SDA---输出
    output              i2c_sda_oe  ,//SDA---输出使能 

    output      [7:0]   dout        ,
    output              done        
);
//状态机参数定义

    localparam  IDLE  = 7'b000_0001,
                START = 7'b000_0010,
                WRITE = 7'b000_0100,
                RACK  = 7'b000_1000,
                READ  = 7'b001_0000,
                SACK  = 7'b010_0000,
                STOP  = 7'b100_0000;

    reg [6:0]   state_c      ;//现态
    reg [6:0]   state_n      ;//次态

    reg [3:0]   cmd_r        ;//命令寄存器
    reg [7:0]   din_r        ;//写数据寄存器
    reg [7:0]   dout_r       ;//输出数据寄存器
    reg         slave_ack_r  ;//接收应答寄存器
//IIC信号寄存
    reg         scl          ;
    reg         sda_out      ;
    reg         sda_out_en   ;   

//比特计数器
    reg [3:0]  cnt_bit       ;
    wire       add_cnt_bit   ;
    wire       end_cnt_bit   ;

//IIC时钟计数器
    reg [7:0]  cnt_scl       ;
    wire       add_cnt_scl   ;
    wire       end_cnt_scl   ;

//状态转移条件
    wire idle2start          ;
    wire idle2write          ;
    wire idle2read           ;
    wire start2write         ;
    wire start2read          ;
    wire write2rack          ;
    wire rack2stop           ;
    wire rack2idle           ;
    wire read2sack           ;
    wire sack2stop           ;
    wire sack2idle           ; 
    wire stop2idle           ;          

//状态机
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_c <= IDLE;
        end
        else
            state_c <= state_n;
    end

    always @(*) begin
        case (state_c)
            IDLE  :begin
                if(idle2start)
                    state_n = START;
                else if (idle2write) begin
                    state_n = WRITE;
                end
                else if (idle2read) begin
                    state_n = READ;
                end
                else
                    state_n = state_c;
            end
            START :begin
                if (start2write) begin
                    state_n = WRITE;
                end
                else if (start2read) begin
                    state_n = READ;
                end
                else
                    state_n = state_c;
            end
            WRITE :begin
                if(write2rack)
                    state_n = RACK ;
                else 
                    state_n = state_c ;
            end
            RACK :begin
                if(rack2stop)
                    state_n = STOP ;
                else if(rack2idle)
                    state_n = IDLE ;
                else 
                    state_n = state_c ;
            end
            READ :begin
                if(read2sack)
                    state_n = SACK ;
                else 
                    state_n = state_c ;
            end
            SACK :begin
                if(sack2stop)
                    state_n = STOP ;
                else if(sack2idle)
                    state_n = IDLE ;
                else 
                    state_n = state_c ;
            end
            STOP :begin
                if(stop2idle)
                    state_n = IDLE ;
                else 
                    state_n = state_c ;
            end
            default : state_n = IDLE;
        endcase
    end
    assign idle2start  = state_c==IDLE  && (req && (cmd & `CMD_START));
    assign idle2write  = state_c==IDLE  && (req && (cmd & `CMD_WRITE)) ;
    assign idle2read   = state_c==IDLE  && (req && (cmd & `CMD_READ)) ;
    assign start2write = state_c==START && (end_cnt_bit && (cmd_r &`CMD_WRITE)) ;
    assign start2read  = state_c==START && (end_cnt_bit && (cmd_r &`CMD_READ)) ;
    assign write2rack  = state_c==WRITE && (end_cnt_bit) ;
    assign read2sack   = state_c==READ  && (end_cnt_bit) ;
    assign rack2stop   = state_c==RACK  && (end_cnt_bit && ((cmd_r & `CMD_STOP) || slave_ack_r));
    assign sack2stop   = state_c==SACK  && (end_cnt_bit && (cmd_r & `CMD_STOP)) ;
    assign rack2idle   = state_c==RACK  && (end_cnt_bit && (cmd_r & `CMD_STOP) == 0) ;
    assign sack2idle   = state_c==SACK  && (end_cnt_bit && (cmd_r & `CMD_STOP) == 0) ;
    assign stop2idle   = state_c==STOP  && (end_cnt_bit) ;

//scl
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl <= 1;
        end
        else if (idle2start | idle2write | idle2read) begin
            scl <= 0;
        end
        else if (add_cnt_scl && cnt_scl == `SCL_HALF-1) begin
            scl <= 1;
        end
        else if (end_cnt_scl && ~stop2idle) begin
            scl <= 0;
        end
    end

//sda_out_en
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_out_en <= 0;
        end
        else if(idle2start | idle2write | read2sack | rack2stop)begin
            sda_out_en <= 1'b1;
        end
        else if(idle2read | start2read | write2rack | stop2idle)begin 
            sda_out_en <= 1'b0;
        end 
    end

//sda_out
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_out <= 1;
        end
    //发起始位
        else if (state_c == START) begin
             if (cnt_scl == `LOW_HLAF) begin    
                sda_out <= 1;       //时钟低电平拉高数据线
             end
             else if (cnt_scl == `HIGH_HALF) begin
                sda_out <= 0;       //时钟高电平拉低数据线---保证可以检测到起始位
             end
        end
    //发数据
        else if (state_c == WRITE) begin
            if (cnt_scl == `LOW_HLAF) begin
                sda_out <= din_r[7-cnt_bit];    //时钟低电平发送数据---并串转换
            end
        end
    //发应答位
        else if (state_c == SACK) begin
            if (cnt_scl ==`LOW_HLAF) begin
                sda_out <= (cmd_r & `CMD_STOP ? 1 : 0);    //时钟低电平发应答位
            end
        end
    //发停止位
        else if (state_c == STOP) begin
            if (cnt_scl == `LOW_HLAF) begin
                sda_out <= 0;       //时钟低电平拉低数据线
            end
            else if (cnt_scl == `HIGH_HALF) begin
                sda_out <= 1;       //时钟高电平拉高数据线---保证可以检测到停止位
            end
        end
    end

//cmd_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_r <= 0;
        end
        else if (req) begin
            cmd_r <= cmd;
        end
    end
//din_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            din_r <= 0;
        end
        else if (req) begin
            din_r <= din;
        end
    end

//cnt_scl
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_scl <= 0;
        end
        else if(add_cnt_scl) begin
            if (end_cnt_scl) begin
                cnt_scl <= 0;
            end
            else
                cnt_scl <= cnt_scl+1;
        end
    end
    
    assign add_cnt_scl = (state_c != IDLE);
    assign end_cnt_scl = add_cnt_scl && cnt_scl == (`SCL_PERIOD) -1;

//cnt_bit
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_bit <= 0;
        end
        else if(add_cnt_bit) begin
            if (end_cnt_bit) begin
                cnt_bit <= 0;
            end
            else
                cnt_bit <= cnt_bit+1;
        end
    end
    
    assign add_cnt_bit = end_cnt_scl;
    assign end_cnt_bit = add_cnt_bit && cnt_bit == ((state_c == READ || state_c == WRITE) ? 8 :1)-1;//除开读写数据需要传输8bit数据，其他都是1bit

//dout_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout_r <= 0;
        end
        else if (state_c == READ) begin
            if (cnt_scl == `HIGH_HALF) begin
                dout_r[7-cnt_bit] <= i2c_sda_i;     //串并转换
            end
        end
    end

//slave_ack_r
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slave_ack_r <= 1;
        end
        else if (state_c == RACK) begin
            if (cnt_scl == `HIGH_HALF) begin
                slave_ack_r <= i2c_sda_i;
            end
        end
    end

    assign i2c_scl = scl;
    assign i2c_sda_o = sda_out;
    assign i2c_sda_oe = sda_out_en;

    //assign slave_ack = slave_ack_r;
    assign dout = dout_r;
    assign done = rack2idle | sack2idle | stop2idle;//1字节数据传输完的标志---每传输1字节都有一个应答位
endmodule
