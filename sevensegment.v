module sevenseg (
    select,
    HEXout
);
    input [4:0] select;
    output reg [0:6] HEXout;
    always @(*)
    begin
        //https://www.dcode.fr/7-segment-display
        // use this site to find decoded 7 segment
        //Binary (Anode) (7bit) for the settings.
        case (select) //case statement
            0 : HEXout = 7'b1000000;
            1 : HEXout = 7'b1111001;
            2 : HEXout = 7'b0100100;
            3 : HEXout = 7'b0110000;
            4 : HEXout = 7'b0011001;
            5 : HEXout = 7'b0010010;
            6 : HEXout = 7'b0000010;
            7 : HEXout = 7'b1111000;
            8 : HEXout = 7'b0000000;
            9 : HEXout = 7'b0010000;
            10 : HEXout = 7'b0001000; //A 
            //11 : HEXout = 7'b1100000; //B
            11 : HEXout = 7'b0000011; //B
            //12 : HEXout = 7'b0110001; //C
            12 : HEXout = 7'b1000110; //C
            //13 : HEXout = 7'b1000010; //D
            13 : HEXout = 7'b0100001; //D
            14 : HEXout = 7'b0000110; //E
            //14 : HEXout = 7'b0110000; //E
            //               ABCDEFG
            //15 : HEXout = 7'b0111000; //F
            15 : HEXout = 7'b0001110; //F
            // 16 show -
            //16 : HEXout = 7'b1111110;
            16 : HEXout = 7'b0111111;
            // 17 C
            17 : HEXout = 7'b1000110; 
            //17 : HEXout = 7'b0110001; 
            // 18 L
            18 : HEXout = 7'b1000111;
            //18 : HEXout = 7'b1110001;
            // 19 S
            19 : HEXout = 7'b0010010;
            //19 : HEXout = 7'b0100100;
            // 20 P
            20 : HEXout = 7'b0001100;
            //20 : HEXout = 7'b0011000;
            // 21 N
            21 : HEXout = 7'b0101011;
            //21 : HEXout = 7'b1101010;
            // 22 only set pin A low
            22 : HEXout = 7'b1111110;
            // 23 only set pin B low
            23 : HEXout = 7'b1111101;
            // 24 only set pin C low
            24 : HEXout = 7'b1111011;
            // 25 only set pin D low
            25 : HEXout = 7'b1110111;
            // 26 only set pin E low
            26 : HEXout = 7'b1101111;
            // 27 only set pin F low
            27 : HEXout = 7'b1011111;
            // 28 only set pin G low
            28 : HEXout = 7'b0111111;
            //switch off 7 HEX5ment character when the bcd digit is not a decimal number.
            default : HEXout = 7'b1111111; 
        endcase
    end
endmodule
//module module1(
//    input clk,
//    input rst,
//    input enable,
//    output reg scl,
//    output reg [7:0] vtg=0,
//    inout sda
//    );
//
//reg [7:0]voltage;
//reg direction=1;  
//reg SDA;
//assign sda=direction?SDA:1'bZ;   
//reg [7:0] state; 
//reg [7:0] address;                  // ADDRESS including read bit
//reg [7:0] count;
//reg ack;
//reg received_bit;
//parameter STATE_IDLE=0,
//          STATE_START=1,
//          STATE_ADD_RW=2,
//          STATE_ACK=3,
//          STATE_DATA=4,
//          STATE_RACK=5,
//          STATE_STOP=6;
//          
//always@(negedge clk)
//begin
//    if(rst==1)  scl<=1;
//    else begin
//             if((state==STATE_IDLE)||(state==STATE_START)||(state==STATE_STOP))  
//                     scl<=1;
//            else    scl<=~scl;
//         end 
//end 
//
//always@(posedge clk)
//begin
//    if(rst==1) begin
//        state<=STATE_IDLE;
//        direction<=1;
//        SDA<=1;
//        address=8'b10100001;  
//        count<=8'd0;      
//       end
//    else begin
//        case(state)
//        
//       STATE_IDLE:  begin                     //idle
//                        direction<=1;
//                        SDA<=1;
//                        if(enable)
//                        state<=STATE_START;
//                     end
//       STATE_START: begin                       //start
//                        SDA<=0;
//                        count<=8;
//                        state<=STATE_ADD_RW;
//                    end
//       STATE_ADD_RW: begin                       //address
//                        if(scl==0) begin
//                        SDA<=address[count-1];
//                        if(count==0)  begin  state<=STATE_ACK; direction<=0;   end
//                        else count<=count-1;
//                        end
//         end
//       STATE_ACK:    begin                       //acknowledge by slave
//                        ack=1;                 //ack<=sda for real time
//                        if(ack==1) begin  state<=STATE_DATA; count<=8;   end
//                        else    begin  state<=STATE_IDLE;   end
//                     end
//       STATE_DATA:  begin                        //start to receive data by slave
//                         if(scl==0) begin           //data is received only when scl is low
//                          received_bit<=sda;
//                          voltage[0]<=received_bit;
//                          voltage<=voltage<<1'b1;
//                          count<=count-1'b1;
//                          if(count==0) begin     state<=STATE_RACK;
//                                                 direction<=1; SDA<=1;      //actual ack
//                                                 vtg<=8'd15; //vtg<=voltage for real time
//                                       end
//                          else    begin    // vtg<=vtg+1'b1;  // for bug test
//                                            state<=STATE_DATA;   end
//                    end
//                    end
//       STATE_RACK:  begin                        //master acknowledging the data sent 
//                       if(scl==0) begin
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
//endmodule