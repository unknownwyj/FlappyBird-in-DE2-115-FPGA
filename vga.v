module VGA(clk,KEY, VGA_HS, VGA_VS ,VGA_R, VGA_G, VGA_B,VGA_BLANK_N,VGA_CLOCK,FL_ADDR,FL_DQ,FL_CE_N,FL_OE_N,FL_RST_N,FL_WE_N,FL_WP_N,SW, IRDA_RXD,
SRAM_ADDR,SRAM_DQ,SRAM_OE_N,SRAM_WE_N,SRAM_CE_N,SRAM_LB_N,SRAM_UB_N,
LEDG,LEDR,
EEP_I2C_SCLK,EEP_I2C_SDAT,
HEX0,HEX1,HEX2,HEX3,HEX6,HEX7,HEX4,HEX5,
I2C_SCLK,I2C_SDAT,AUD_XCK,AUD_BCLK,AUD_DACDAT,AUD_DACLRCK,
LCD_EN, LCD_RS, LCD_RW, LCD_DATA,
SD_CLK,SD_CMD,SD_DAT,debug);
output [6:0] HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7;
input [3:0]KEY;
output [17:0]LEDR;
output [7:0]LEDG;
wire rst;
assign rst = KEY[0]; 
input  IRDA_RXD;
input clk;		//clk 50MHz
output VGA_HS, VGA_VS;
output [7:0] VGA_R,VGA_G,VGA_B;
output VGA_BLANK_N,VGA_CLOCK;

reg VGA_HS, VGA_VS;
reg[10:0] counterHS;
reg[9:0] counterVS;
reg [2:0] valid;
reg clk25M;
reg collition;
assign LEDR[1] = collition;
reg signed [13:0] X,Y;
//? FLASH STUFF
output FL_CE_N;
output FL_OE_N; // 
output FL_RST_N; // always high
output FL_WE_N; // always high
output FL_WP_N; // not used
input [7:0] FL_DQ;
output [22:0] FL_ADDR;
assign FL_WE_N = 1'b1;
assign FL_RST_N = 1'b1;
assign FL_CE_N = 1'b0; // always read
assign FL_OE_N = 1'b0; // always read
//? FLASH STUFF END
//? LCD
// lcd used for Audio Volume control view
output reg LCD_EN, LCD_RS, LCD_RW;
inout [7:0] LCD_DATA;
wire            clk400;
div400 	div_400HZ(.clk50M(clk), .rst(rst), .clk400(clk400));
reg     [3:0]   	state, next_command;
reg     [4:0]   	CHAR_COUNT;
reg     [7:0] 	    DATA_BUS_VALUE;
wire    [255:0] 	ID_data;
reg     [127:0] 	num_Data;
wire    [127:0] 	org_num;
reg     [127:0] 	crypto_num;
reg 	[5:0] 	    font_addr;
wire 	[7:0]		font_data;
wire    [7:0]   	Next_Char;
assign LCD_DATA = LCD_RW ? 8'bZZZZZZZZ : DATA_BUS_VALUE;
//////////////////////LCD_state//////////////////////
//////////////////////DONT MOVE/////////////////////////
    parameter  RESET 			= 4'd0,
	            DROP_LCD_E 		= 4'd1,
	            HOLD 			= 4'd2,
	            DISPLAY_CLEAR 	= 4'd3,
	            MODE_SET 		= 4'd4,
	            Print_String 	= 4'd5,
	            LINE2 			= 4'd6,
	            RETURN_HOME 	= 4'd7,
	            CG_RAM_HOME 	= 4'd8,
	            write_CG 		= 4'd9;

    always @(posedge clk400 or negedge rst)begin
		if (!rst)begin
			state <= RESET;
		end
		else begin
			case (state)
				RESET : begin  // Set Function to 8-bit transfer and 2 line display with 5x8 Font size
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'h38;
					state 			<= DROP_LCD_E;
					next_command 	<= DISPLAY_CLEAR;
					CHAR_COUNT 		<= 5'b00000;
				end

				// Clear Display (also clear DDRAM content)
				DISPLAY_CLEAR : begin
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'h01;
					state 			<= DROP_LCD_E;
					next_command 	<= MODE_SET;
				end

				// Set write mode to auto increment address and move cursor to the right
				MODE_SET : begin
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'h06;
					state 			<= DROP_LCD_E;
					next_command 	<= CG_RAM_HOME;
				end

				// Write ASCII hex character in first LCD character location
				Print_String : begin
					state 			<= DROP_LCD_E;
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b1;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= Next_Char;
					
					// Loop to send out 32 characters to LCD Display  (16 by 2 lines)
					if (CHAR_COUNT < 31)
						CHAR_COUNT <= CHAR_COUNT + 1'b1;
					else
						CHAR_COUNT <= 5'b00000; 

					// Jump to second line?
					if (CHAR_COUNT == 15)
						next_command <= LINE2;
					// Return to first line?
					else if (CHAR_COUNT == 31)
						next_command <= RETURN_HOME;
					else
						next_command <= Print_String;
				end

				// Set write address to line 2 character 1
				LINE2 : begin
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'hC0; //line 2 character 2 ==> 8'hC1
					state 			<= DROP_LCD_E;
					next_command 	<= Print_String;
				end

				// Return write address to first character postion on line 1
				RETURN_HOME : begin
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'h80; //line 1 character 2 ==> 8'h81
					state 			<= DROP_LCD_E;
					next_command 	<= Print_String;
				end
				CG_RAM_HOME : begin
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b0;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= 8'h40; //CGRAM begin address = 6'h00
					font_addr		<= 6'd0;//
					state 			<= DROP_LCD_E;
					next_command 	<= write_CG;
				end
				write_CG : begin
					state 			<= DROP_LCD_E;
					LCD_EN 			<= 1'b1;
					LCD_RS 			<= 1'b1;
					LCD_RW 			<= 1'b0;
					DATA_BUS_VALUE 	<= font_data;

					if(font_addr == 6'b111111)begin
						next_command 	<= RETURN_HOME;
					end else begin
						font_addr		<= font_addr + 6'b1;
						next_command 	<= write_CG;
					end
				end

				DROP_LCD_E : begin
					LCD_EN 	<= 1'b0;
					state 	<= HOLD;
				end
				
				HOLD : begin
					state 	<= next_command;
				end
			endcase
		end
	end
////////////////////////////////////////////////////
Custom_font_ROM cr(.addr(font_addr), .out_data(font_data));
LCD_display_string LCD( .clk(clk), .rst(rst), 
.index(CHAR_COUNT), .ID_data(ID_data), .out(Next_Char));
parameter   lcdspace   =   8'h20,lcdA=8'h41,lcdB=8'h42,lcdC=8'h43,lcdD=8'h44,lcdE=8'h45,lcdF=8'h46,lcdG=8'h47,lcdH=8'h48,lcdI=8'h49,lcdJ=8'h4A,lcdK=8'h4B,lcdL=8'h4C,lcdM=8'h4D,lcdN=8'h4E,lcdO=8'h4F,lcdP=8'h50,lcdQ=8'h51,lcdR=8'h52,lcdS=8'h53,lcdT=8'h54,lcdU=8'h55,lcdV=8'h56,lcdW=8'h57,lcdX=8'h58,lcdY=8'h59,lcdZ=8'h5A,
            lcdzero    =   8'h30,
            lcdone     =   8'h31,
            lcdtwo     =   8'h32,
            lcdthree   =   8'h33,
            lcdfour    =   8'h34,
            lcdfive    =   8'h35,
            lcdsix     =   8'h36,
            lcdseven   =   8'h37,
            lcdeight   =   8'h38,
            lcdnine    =   8'h39,
            lcdpointleft = 8'b00111100,
            lcdpointright= 8'b00111110,
            lcdmiddleline = 8'b01111100,
            lcdfill    =   8'hFF;
reg [7:0] mappedoutdisplay [0:35];
assign ID_data[255:248] = mappedoutdisplay[0];
assign ID_data[247:240] = mappedoutdisplay[1];
assign ID_data[239:232] = mappedoutdisplay[2];
assign ID_data[231:224] = mappedoutdisplay[3];
assign ID_data[223:216] = mappedoutdisplay[4];
assign ID_data[215:208] = mappedoutdisplay[5];
assign ID_data[207:200] = mappedoutdisplay[6];
assign ID_data[199:192] = mappedoutdisplay[7];
assign ID_data[191:184] = mappedoutdisplay[8];
assign ID_data[183:176] = mappedoutdisplay[9];
assign ID_data[175:168] = mappedoutdisplay[10];
assign ID_data[167:160] = mappedoutdisplay[11];
assign ID_data[159:152] = mappedoutdisplay[12];
assign ID_data[151:144] = mappedoutdisplay[13];
assign ID_data[143:136] = mappedoutdisplay[14];
assign ID_data[135:128] = mappedoutdisplay[15];
assign ID_data[127:120] = mappedoutdisplay[20];
assign ID_data[119:112] = mappedoutdisplay[21];
assign ID_data[111:104] = mappedoutdisplay[22];
assign ID_data[103:96] = mappedoutdisplay[23];
assign ID_data[95:88] = mappedoutdisplay[24];
assign ID_data[87:80] = mappedoutdisplay[25];
assign ID_data[79:72] = mappedoutdisplay[26];
assign ID_data[71:64] = mappedoutdisplay[27];
assign ID_data[63:56] = mappedoutdisplay[28];
assign ID_data[55:48] = mappedoutdisplay[29];
assign ID_data[47:40] = mappedoutdisplay[30];
assign ID_data[39:32] = mappedoutdisplay[31];
assign ID_data[31:24] = mappedoutdisplay[32];
assign ID_data[23:16] = mappedoutdisplay[33];
assign ID_data[15:8] =  mappedoutdisplay[34];
assign ID_data[7:0] =   mappedoutdisplay[35];
integer loop;
// 0123456789ABCDEF
//   SFX    >BGM<
// XXXXXXX|XXXXXXX
reg selectaudioControl;
parameter   audioControlSFX = 0,
            audioControlBGM = 1;
always @(*) begin
    for(loop = 0; loop < 16;loop = loop + 1)begin
        mappedoutdisplay[loop] = lcdspace;
    end
    for(loop = 20; loop < 36;loop = loop + 1)begin
        mappedoutdisplay[loop] = lcdspace;
    end
    for(loop = 0; loop < SFXvolume; loop = loop + 1)begin
        mappedoutdisplay[loop] = lcdfill;
    end
    for(loop = 8; loop < 8+BGMvolume; loop = loop + 1)begin
        mappedoutdisplay[loop] = lcdfill;
    end
    mappedoutdisplay[7] = lcdmiddleline;
    mappedoutdisplay[22] = lcdS;
    mappedoutdisplay[23] = lcdF;
    mappedoutdisplay[24] = lcdX;
    mappedoutdisplay[30] = lcdB;
    mappedoutdisplay[31] = lcdG;
    mappedoutdisplay[32] = lcdM;
    if(selectaudioControl == audioControlBGM)begin
        mappedoutdisplay[29] = lcdpointright;
        mappedoutdisplay[33] = lcdpointleft;
    end
    else begin
        mappedoutdisplay[21] = lcdpointright;
        mappedoutdisplay[25] = lcdpointleft;
    end
end
//? LCD END
//? AUDIO ???????? NANI
output I2C_SCLK;
inout I2C_SDAT;
output AUD_XCK;
output AUD_BCLK;
output reg AUD_DACDAT;
output reg AUD_DACLRCK;
reg [3:0] audio_reg_counter; //selecting register address and its corresponding data
reg counting_state,ignition,audioROMreadenable; 	
reg [15:0] MUX_input;
reg [3:0] ROM_output_mux_counter;
reg [4:0] AudioBGMstereoCounter;
reg [4:0] DAC_LR_CLK_counter;
wire finish_flag;
wire [2:0]ACK_LEDR;
assign LEDR[5:3] = ACK_LEDR;
//assign AUD_DACDAT = (SW[5])?AudioPointout[15-ROM_output_mux_counter]:0;
reg signed [15:0] sfxmixed;
reg signed [15:0] realaudio;
// max 3 audio source 2 sfx and 1 bgm
reg [2:0] SFXvolume,BGMvolume; // 0-7 7 is loudest
reg signed [15:0] AudioSFX1,AudioSFX2;
reg signed [15:0] AudioBGM; // background music should only have 1
//assign AudioBGM = (AudioBGMout*(BGMvolume+1))/7;
output reg [15:0]debug;
always@(*)begin
    case(BGMvolume)
    0: AudioBGM = 0;
    //1: AudioBGM = (AudioBGMout<<<6)>>>12;
    //2: AudioBGM = (AudioBGMout<<<7)>>>12;
    //3: AudioBGM = (AudioBGMout<<<8)>>>12;
    4: AudioBGM = {SD_AudioData[0],SD_AudioData[1],SD_AudioData[2],SD_AudioData[3],SD_AudioData[4],SD_AudioData[5],SD_AudioData[6],SD_AudioData[7],SD_AudioData[8],SD_AudioData[9],SD_AudioData[10],SD_AudioData[11],SD_AudioData[12],SD_AudioData[13],SD_AudioData[14],SD_AudioData[15]};//(AudioBGMout<<<9)>>>12;
    5: AudioBGM = {SD_AudioData[15],SD_AudioData[14],SD_AudioData[13],SD_AudioData[12],SD_AudioData[11],SD_AudioData[10],SD_AudioData[9],SD_AudioData[8],SD_AudioData[7],SD_AudioData[6],SD_AudioData[5],SD_AudioData[4],SD_AudioData[3],SD_AudioData[2],SD_AudioData[1],SD_AudioData[0]};//(AudioBGMout<<<10)>>>12;
    6: AudioBGM = SD_AudioData[15:0];
    7: AudioBGM = ({SD_AudioData[7:0],SD_AudioData[15:8]});
    default: AudioBGM = SD_AudioData;
    endcase
end
always @(*)begin // https://www.vttoth.com/CMS/technical-notes/?view=article&id=68
    if(AudioDieROMADDR != 0) // point and die tgt point lower priority
    begin
        AudioSFX1 = (AudioDieout);//<<<(SFXvolume+1))>>>8;
    end
    else if(AudioPointROMADDR != 0)begin
        AudioSFX1 = (AudioPointout);//<<<(SFXvolume+1))>>>8;
    end
    else begin
        AudioSFX1 = 0;
    end
    //Wing sound effect
    if(AudioWingROMADDR != 0)begin
        AudioSFX2 = (AudioWingout);//<<<(SFXvolume+1))>>>8;
    end
    else begin
        AudioSFX2 = 0;
    end
    //BGM
    sfxmixed = AudioSFX1 + AudioSFX2 - (AudioSFX1*AudioSFX2)/65535; // split into two
    realaudio = sfxmixed + AudioBGM - (sfxmixed*AudioBGM)/65535; // split into two
	//realaudio = SD_AudioData;
    //realaudio = AudioSFX1 + AudioSFX2 + AudioBGM - (AudioSFX1*AudioSFX2)/65535 - (AudioSFX1*AudioBGM)/65535 - (AudioSFX2*AudioBGM)/65535 + (AudioSFX1*AudioSFX2*AudioBGM)/(4294836225);
end
always @(*)begin
    if(SW[5])begin
        AUD_DACDAT = 0;
    end
    else begin
        AUD_DACDAT = realaudio[15-ROM_output_mux_counter];
        //AUD_DACDAT = SD_AudioData[31-DAC_LR_CLK_counter];
    end
end
//Instantiation s
I2C_Protocol I2C(
	.clk(clk),
	.reset(rst),
	.ignition(ignition),
	.MUX_input(MUX_input),
	.ACK(ACK_LEDR),
	.SDIN(I2C_SDAT),
	.finish_flag(finish_flag),
	.SCLK(I2C_SCLK)
);
pll	pll1 (
	.inclk0 ( clk ),
	.c0 ( AUD_XCK ),
	.c1 ( AUD_BCLK )
	);
reg [12:0] AudioPointROMADDR;
wire signed [15:0]AudioPointout;
parameter AudioPointSize = 8000;
AudioPoint_ROM	PointROM (
	.address ( AudioPointROMADDR ),
	.clock ( clk ),
	.rden ( audioROMreadenable ), // put same on every ROM
	.q ( AudioPointout )
);
reg [13:0] AudioDieROMADDR;
wire signed [15:0]AudioDieout; //! hit and die tgt.
parameter AudioDieSize = 10766;
AudioDie_ROM DieROM(
    .address ( AudioDieROMADDR ),
    .clock ( clk ),
    .rden ( audioROMreadenable ), // put same on every ROM
    .q ( AudioDieout )
);
reg [11:0] AudioWingROMADDR;
wire signed [15:0]AudioWingout;
parameter AudioWingSize = 2700;//2700 no sound 2705 real 2758 behind got useless suffix
AudioWing_ROM WingROM(
    .address ( AudioWingROMADDR ),
    .clock ( clk ),
    .rden ( audioROMreadenable ), // put same on every ROM
    .q ( AudioWingout )
);
reg [16:0] AudioBGMROMADDR;
wire signed [31:0]AudioBGMout;
// not sure yet maybe will be outside rom or sdcard to sram.
// if sdcard then need FIFO to store the data
parameter AudioBGMSize = 89529; // 89589 got cacat sound at the end we remove some
AudioBGM_ROM BGMROM(
    .address ( AudioBGMROMADDR ),
    .clock ( clk ),
    .rden ( audioROMreadenable ), // put same on every ROM
    .q ( AudioBGMout )
);
// address for ROM
reg [7:0]playsoundtrig;
always @(posedge clk or negedge rst) begin
    if(!rst)
    begin
        playsoundtrig <= 0;
    end
    else begin
        if(!KEY[2])begin
            playsoundtrig <= playsoundtrig + 1'b1;
        end
        else begin
            playsoundtrig <= playsoundtrig;
        end
    end
end
reg half;
reg [7:0]trigreg;
reg [3:0]jumptriggercheck;
reg [7:0]currentscoresound;
reg playdiesound;
always @(posedge AUD_DACLRCK,negedge rst)begin
	if(!rst)begin
		half <=1;
        AudioPointROMADDR <= 0;
        AudioDieROMADDR <= 0;
        trigreg <= 0;
        jumptriggercheck <= 0;
        currentscoresound <= 0;
        playdiesound <= 1;
	end
	else if(audioROMreadenable) 
	begin
		if(half == 1)begin
            // every 2 clock cycle increment the address cause we only single channel
		    //AudioPointROMADDR <= AudioPointROMADDR + 1;
		    //if (AudioPointROMADDR == AudioPointSize)begin
            //    AudioPointROMADDR <= 0;
            //end
            //if (AudioDieROMADDR != 0)begin
            //    AudioDieROMADDR <= AudioDieROMADDR + 1;
            //end
            //if(AudioDieROMADDR == AudioDieSize)begin
            //    AudioDieROMADDR <= 0;
            //end
            //if(playsoundtrig != trigreg)begin
            //    trigreg <= playsoundtrig;
            //    AudioDieROMADDR <= 1;
            //end
            if(AudioBGMROMADDR == AudioBGMSize)begin
                AudioBGMROMADDR <= 0;
            end
            else begin
                AudioBGMROMADDR <= AudioBGMROMADDR + 1;
            end
            if(currentstate == ingame)begin
                playdiesound <= 1;
                if(AudioDieROMADDR != 0)begin
                    AudioDieROMADDR <= 0;
                end
                if(scorecount != currentscoresound && ROM_output_mux_counter == 15)begin
                    currentscoresound <= scorecount;
                    AudioPointROMADDR <= 1;
                end
                else if (AudioPointROMADDR == AudioPointSize)begin
                    AudioPointROMADDR <= 0;
                end
                else if(AudioPointROMADDR != 0)begin
                    AudioPointROMADDR <= AudioPointROMADDR + 1;
                end
                if(jumptriggercheck != jumptriggered && ROM_output_mux_counter == 15)begin // to prevent popping sound ?
                    jumptriggercheck <= jumptriggered;
                    AudioWingROMADDR <= 1;
                end
                else if(AudioWingROMADDR == AudioWingSize)begin
                    AudioWingROMADDR <= 0;
                end
                else if(AudioWingROMADDR != 0)begin
                    AudioWingROMADDR <= AudioWingROMADDR + 1;
                end
            end
            else if(currentstate == gameover)
            begin
                if(playdiesound)begin
                    AudioDieROMADDR <= 1;
                    playdiesound <= 0;
                end
                else if(AudioDieROMADDR == AudioDieSize)begin
                    AudioDieROMADDR <= 0;
                end
                else if(AudioDieROMADDR != 0)begin
                    AudioDieROMADDR <= AudioDieROMADDR + 1;
                end
                if(AudioWingROMADDR == AudioWingSize)begin
                    AudioWingROMADDR <= 0;
                end
                else if(AudioWingROMADDR != 0)begin
                    AudioWingROMADDR <= AudioWingROMADDR + 1;
                end
                if(AudioPointROMADDR == AudioPointSize)begin
                    AudioPointROMADDR <= 0;
                end
                else if(AudioPointROMADDR != 0)begin
                    AudioPointROMADDR <= AudioPointROMADDR + 1;
                end
                if(AudioDieROMADDR == AudioDieSize)begin
                    AudioDieROMADDR <= 0;
                end
                else if(AudioDieROMADDR != 0)begin
                    AudioDieROMADDR <= AudioDieROMADDR + 1;
                end
            end
            else
            begin
                if(AudioWingROMADDR == AudioWingSize)begin
                    AudioWingROMADDR <= 0;
                end
                else if(AudioWingROMADDR != 0)begin
                    AudioWingROMADDR <= AudioWingROMADDR + 1;
                end
                if(AudioPointROMADDR == AudioPointSize)begin
                    AudioPointROMADDR <= 0;
                end
                else if(AudioPointROMADDR != 0)begin
                    AudioPointROMADDR <= AudioPointROMADDR + 1;
                end
                if(AudioDieROMADDR == AudioDieSize)begin
                    AudioDieROMADDR <= 0;
                end
                else if(AudioDieROMADDR != 0)begin
                    AudioDieROMADDR <= AudioDieROMADDR + 1;
                end
                currentscoresound <= 0;
            end
        end
	end
end
// ROM output mux
//always @(posedge AUD_BCLK) 
//	begin
//	if(audioROMreadenable)
//		begin
//		ROM_output_mux_counter <= ROM_output_mux_counter + 1;
//		if (DAC_LR_CLK_counter == 31) AUD_DACLRCK <= 1;
//		else AUD_DACLRCK <= 0;
//		end
//	end
reg audioclk2;
reg anotherhalfclk;
always @(posedge AUD_BCLK) 
	begin
	if(audioROMreadenable)
		begin
		ROM_output_mux_counter <= ROM_output_mux_counter + 1;
		if (DAC_LR_CLK_counter == 31) AUD_DACLRCK <= 1;
		else AUD_DACLRCK <= 0;
        if (DAC_LR_CLK_counter == SW[17:17-3]) 
            if(!anotherhalfclk && SW[17-4])
            begin
                anotherhalfclk <= 1;
            end
            else 
            begin
                anotherhalfclk <= 0;
                audioclk2 <= 1;
            end
        else 
            audioclk2 <= 0;
		end
	end
always @(posedge AUD_BCLK)
	begin
	if(audioROMreadenable)
		begin
		DAC_LR_CLK_counter <= DAC_LR_CLK_counter + 1;
		end
	end
//============================================
// generate 6 configuration pulses 
always @(posedge clk)
	begin
	if(!rst) 
		begin
		counting_state <= 0;
		audioROMreadenable <= 0;
		end
	else
		begin
		case(counting_state)
		0:
			begin
			ignition <= 1;
			audioROMreadenable <= 0;
			if(audio_reg_counter == 8) counting_state <= 1; //was 8
			end
		1:
			begin
			audioROMreadenable <= 1;
			ignition <= 0;
			end
		endcase
		end
	end
// this counter is used to switch between registers
always @(posedge I2C_SCLK)
	begin
		case(audio_reg_counter) //MUX_input[15:9] register address, MUX_input[8:0] register data
		//0: MUX_input <= 16'h1201; // activate interface
		//1: MUX_input <= 16'b0000_0100_01_010001; // left headphone out
		//2: MUX_input <= 16'h0C00; // power down control
		//3: MUX_input <= 16'h0812; // analog audio path control
		//4: MUX_input <= 16'h0A00; // digital audio path control
		//5: MUX_input <= 16'h102F; // sampling control
        //6: MUX_input <= 16'h0E23; // digital audio interface format
		////6: MUX_input <= 16'b00001110_0001_0011; // digital audio interface format
		//7: MUX_input <= 16'h0660; // right headphone out
		//8: MUX_input <= 16'h1E00; // rst device
        0: MUX_input <= 16'h1201; // activate interface
		1: MUX_input <= 16'h0460; // left headphone out
		2: MUX_input <= 16'h0C00; // power down control
		3: MUX_input <= 16'h0812; // analog audio path control
		4: MUX_input <= 16'h0A00; // digital audio path control
		5: MUX_input <= 16'h102F; // sampling control
		6: MUX_input <= 16'h0E23; // digital audio interface format
		7: MUX_input <= 16'h0660; // right headphone out
		8: MUX_input <= 16'h1E00; // reset device
		endcase
	end
always @(posedge finish_flag or negedge rst)
begin
    if(!rst)
        audio_reg_counter <= 0;
        else 
        audio_reg_counter <= audio_reg_counter + 1;
end

//? EEPROM STUFF
// 400khz wanted on FI2Cclk
//prescale
//   set prescale to 1/4 of the minimum clock period in units
//   of input clk cycles (prescale = Fclk / (FI2Cclk * 4))
//   for 50MHz input clock and 400kHz I2C clock, prescale = 31
// to debug i2c use hex
inout EEP_I2C_SCLK;
inout EEP_I2C_SDAT;
assign LEDR[17] = firetagged[1];
assign LEDR[16] = firetagged[0];
sevenseg sevenseg0(highscorefromeeprom[3:0],HEX4);
sevenseg sevenseg1(highscorefromeeprom[7:4],HEX5);
sevenseg sevenseg2(highscoretoeeprom[3:0],HEX6);
sevenseg sevenseg3(highscoretoeeprom[7:4],HEX7);
wire eeprom_valid;
reg starteepromwriteorread;
reg [15:0]eepromaddr;
reg LreadHwrite;
reg [7:0] highscoretoeeprom;
wire [7:0] highscorefromeeprom;
reg updatescore;
i2c_eeprom i2c_eeprom0(
    .clk(clk),
    .rst_n(rst),
    .EEP_I2C_SDAT(EEP_I2C_SDAT),
    .EEP_I2C_SCLK(EEP_I2C_SCLK),
    .data_in(highscoretoeeprom),
    .enable(starteepromwriteorread),
    .LreadHwrite(LreadHwrite),
    .valid(eeprom_valid),
    .data_out(highscorefromeeprom),
    .addr_in(eepromaddr)
    );
reg [3:0] eepromstate;
localparam  S_RST = 0,
            S_INITIALREAD = 1,
            S_IDLE = 2,
            S_WRITE = 3;
always@(posedge clk or negedge rst)begin
    if(!rst)begin
        starteepromwriteorread <= 0;
        eepromaddr <= 0;
        LreadHwrite <= 0;
        eepromstate <= S_RST;
    end
    else begin
        case(eepromstate)
        S_RST:begin
            // after reset read the highscore from eeprom
            eepromstate <= S_INITIALREAD;
            LreadHwrite <= 0;// read
            starteepromwriteorread <= 1'b1;
            eepromaddr <= 16'h0000; 
        end
        S_INITIALREAD:begin
            if(eeprom_valid)begin
                eepromstate <= S_IDLE;
            end
            else begin
                eepromstate <= S_INITIALREAD;
                starteepromwriteorread <= 0; // only need to trigger
            end
        end
        S_IDLE:begin
            if(updatescore)begin
                eepromstate <= S_WRITE;
                LreadHwrite <= 1'b1; // write
                starteepromwriteorread <= 1'b1;
                eepromaddr <= 16'h0000;
            end
            else begin
                eepromstate <= S_IDLE;
                starteepromwriteorread <= 0;
            end
        end
        S_WRITE:begin
            if(eeprom_valid)begin
                eepromstate <= S_IDLE;
            end
            else begin
                eepromstate <= S_WRITE;
                starteepromwriteorread <= 0;
            end
        end
        endcase
    end
end
//? EEPROM STUFF END
//? SRAM STUFF
output wire [19:0]SRAM_ADDR;
inout reg [15:0]SRAM_DQ;
output reg SRAM_OE_N;
output reg SRAM_WE_N;
output reg SRAM_CE_N;
output wire SRAM_LB_N;
output wire SRAM_UB_N;
assign SRAM_LB_N = 1'b0; // always enable lower byte
assign SRAM_UB_N = 1'b0; // always enable upper byte
//? SRAM STUFF END
parameter H_FRONT = 16;										// state E
parameter H_SYNC  = 96;										// state B
parameter H_BACK  = 48;										// state C
parameter H_ACT   = 640;									// state D
parameter H_BLANK = H_FRONT + H_SYNC + H_BACK;
parameter H_TOTAL = H_FRONT + H_SYNC + H_BACK + H_ACT;

parameter V_FRONT = 11;										// state S
parameter V_SYNC  = 2;										// state P
parameter V_BACK  = 32;										// state Q
parameter V_ACT   = 480;									// state R
parameter V_BLANK = V_FRONT + V_SYNC + V_BACK;
parameter V_TOTAL = V_FRONT + V_SYNC + V_BACK + V_ACT;
assign VGA_SYNC_N = 1'b0;
assign VGA_BLANK_N = ~((counterHS<H_BLANK)||(counterVS<V_BLANK));
assign VGA_CLOCK = ~clk25M;
// * CORDIC rotation stuff
//! NOT YET IMPLEMENTED
//cordic CORDIC(
//    .clk(clk),
//    .rst(~rst),
//    .x_i(17'd19896),
//    .y_i(16'd0),
//    .theta_i(cordic_angle),
//    .x_o(COS),
//    .y_o(SIN),
//    .theta_o(),
//    .valid_in(),
//    .valid_out()
//    );



// * CORDIC END
parameter Xaxisstart = 176;
// ROM
reg [14:0] ROMBASE_ADDR;
wire [7:0] ROMBASE_OUT;
romBase platformBase(ROMBASE_ADDR,clk,ROMBASE_OUT);
parameter romBASEwidth = 288;
parameter romBASEheight = 80;

//Yellow Bird.
reg [11:0] ROMYBIRD_ADDR;
wire [7:0] ROMYBIRD_OUT;
romYbird Yellowbird(ROMYBIRD_ADDR,clk,ROMYBIRD_OUT);
parameter romYBIRDwidth = 35; // wtf this wrong eh 34
parameter romYBIRDheight = 24; // wtf this wrong eh 24
parameter romYBIRDupoffset = 0;
parameter romYBIRDmidoffset = 816; // *x to get next bird
parameter romYBIRDdownoffset = 1632;
//change color
wire [7:0] ROMYBIRD_map[255];
wire [1:0] selectbirdC;
assign selectbirdC = SW[4:3];
assign ROMYBIRD_map[96] = (selectbirdC == 1) ? 8'd25 : (selectbirdC == 2) ? 8'd9 : 8'd96;
assign ROMYBIRD_map[24] = (selectbirdC == 1) ? 8'd22 : (selectbirdC == 2) ? 8'd5 : 8'd24;
assign ROMYBIRD_map[91] = (selectbirdC == 1) ? 8'd20 : (selectbirdC == 2) ? 8'd3 : 8'd91;
assign ROMYBIRD_map[22] = 22;
assign ROMYBIRD_map[2] = 2;
assign ROMYBIRD_map[6] =6;
assign ROMYBIRD_map[4] =4;
assign ROMYBIRD_map[179] = 8'd179;
// 0 = y, 1 = r, 2 = b, 3 = y
// b  r  y
// 3 20 91
// 5 22 24
// 9 25 96
// Flame Hex
reg [13:0] ROMFLAME_ADDR;
wire [7:0] ROMFLAME_OUT;
ROMflame Flame(ROMFLAME_ADDR,clk,ROMFLAME_OUT);
parameter romFLAMEwidth = 127;
parameter romFLAMEheight = 70;
// GREEN PIPE
reg [14:0] ROMGREENPIPE_ADDR;
wire [7:0] ROMGREENPIPE_OUT;
ROMpipegreen GreenPipe(ROMGREENPIPE_ADDR,clk,ROMGREENPIPE_OUT);
parameter romGREENPIPEwidth = 52;
parameter romGREENPIPEheight = 320;
parameter pipegap = 140;
// NUMBERS
reg [13:0] ROMNUMBERS_ADDR;
wire [7:0] ROMNUMBERS_OUT;
ROMnumbers Numbers(ROMNUMBERS_ADDR,clk,ROMNUMBERS_OUT);
parameter romNUMBERSwidth = 24;
parameter romNUMBERSheight = 36;
parameter romNUMBERSoffset = 864; // *x to get next number
parameter romNUMBERSheightoffset = 10;
parameter romNUMBERSbetweennumberGAP = 4;
// MEDAL
reg [12:0] ROMMEDAL_ADDR;
wire [7:0] ROMMEDAL_OUT;
medalROM Medal(ROMMEDAL_ADDR,clk,ROMMEDAL_OUT);
parameter romMEDALwidth = 44;
parameter romMEDALheight = 44;
parameter romMEDALoffset = 1936; // *x to get next medal
parameter   medalnone = 0,
            medalbronze = 1,
            medalsilver = 2,
            medalgold = 3,
            medalplatinum = 4;
reg [2:0] medaltype;
//SCOREboard
reg [14:0] SCOREBOARD_ADDR;
wire [7:0] SCOREBOARD_OUT;
scoreboard Scoreboard(SCOREBOARD_ADDR,clk,SCOREBOARD_OUT);
parameter romSCOREBOARDwidth = 226;
parameter romSCOREBOARDheight = 114;
//smallnumber
reg [10:0] ROMSMALLNUMBER_ADDR;
wire [7:0] ROMSMALLNUMBER_OUT;
ROMsmallnumber Smallnumber(ROMSMALLNUMBER_ADDR,clk,ROMSMALLNUMBER_OUT);
parameter romSMALLNUMBERwidth = 12;
parameter romSMALLNUMBERheight = 14;
parameter romSMALLNUMBERoffset = 168; // *x to get next number

//FlameReady Rom
parameter FlameReadywidth = 122;
parameter FlameReadyheight = 39;
parameter FlameReadyCutoffy = 12; 
reg [12:0] ROMFLAMEREADY_ADDR;
wire [0:0] ROMFLAMEREADY_OUT;
FlameReadySign Flameready(ROMFLAMEREADY_ADDR,clk,ROMFLAMEREADY_OUT);

// Flame Bar ROM
parameter FlameBarwidth = 136;
parameter FlameBarheight = 9;
parameter FlameBarExtrawidth = 16;
reg [10:0] ROMFLAMEREADYBAR_ADDR;
wire [0:0] ROMFLAMEREADYBAR_OUT;
FlameReadybarROM Flamebar(ROMFLAMEREADYBAR_ADDR,clk,ROMFLAMEREADYBAR_OUT);

// ENDROM
input [17:0]SW;
wire [7:0]length;
assign length = SW[7:0];
always@(posedge clk)
	clk25M = ~clk25M;

// DON'T MOVE
always@(posedge clk25M)begin
	if(!rst) 
		counterHS <= 0;
	else begin
		if(counterHS == H_TOTAL) 
			counterHS <= 0;
		else 
			counterHS <= counterHS + 1'b1;
		
		if(counterHS == H_FRONT-1)
			VGA_HS <= 1'b0;
		if(counterHS == H_FRONT + H_SYNC -1)
			VGA_HS <= 1'b1;
			
		if(counterHS >= H_BLANK)							// record X position
			X <= counterHS-H_BLANK;
		else
			X <= 0;	
	end
end

// DON'T MOVE
always@(posedge clk25M)begin
	if(!rst) 
		counterVS <= 0;
	else begin
	
		if(counterVS == V_TOTAL) 
			counterVS <= 0;
		else if(counterHS == H_TOTAL) 
			counterVS <= counterVS + 1'b1;
			
		if(counterVS == V_FRONT-1)
			VGA_VS <= 1'b0;
		if(counterVS == V_FRONT + V_SYNC -1)
			VGA_VS <= 1'b1;
		if(counterVS >= V_BLANK)							// record Y position
			Y <= counterVS-V_BLANK;
		else
			Y <= 0;
	end
end
assign FL_ADDR = externalromaddr; // connected to mux because need to load asset to sram
reg [23:0]color[3:0];
parameter   Gameoverwidth = 184,
            Gameoverheight = 267;
reg [7:0] selectcolor; // if == 179 then is background

decodecolor decodecolor0(
    .color(selectcolor),
    .red(VGA_R),
    .green(VGA_G),
    .blue(VGA_B)
);
reg [22:0]counter;
reg [12:0]oldX;
reg [12:0]oldY;
reg [0:0] physicsclk;
wire [0:0] enablenightmode;
assign enablenightmode = SW[0];
reg [0:0] nightmodereg;
assign LEDG[7:0] = scorecount;
//assign LEDR[17:6] = ROMNUMBERS_ADDR;
// used to multiplex between the background read and the asset write
wire [22:0]externalromaddr;
assign externalromaddr = (currentstate == loadasset) ? assetloadaddr : counter;

reg [7:0] nextcolor;
reg [3:0] currentstate;
reg [3:0] nextstate;
parameter   loadasset = 0,
            waitforstart = 1,
            ingame = 2,
            gameover = 3;

parameter   setupSRAM = 0,
            setSRaddr = 1,
            setSRdata = 2,
            setSRdone = 3;

parameter   getreadywidth = 184,
            getreadyheight = 267;
reg [22:0] assetloadaddr;
reg [19:0] readSRAMaddr;
reg [19:0] writeSRAMaddr;
assign SRAM_ADDR = (currentstate == loadasset) ? writeSRAMaddr : readSRAMaddr;
reg [3:0] sramwritestate;
parameter endofassetaddr = 23'h47FFF + 32'd49128;
// event to control the state machine.
always @(*)begin
    case (currentstate)
    loadasset:
    begin
        if(assetloadaddr == endofassetaddr + 1)begin
            nextstate <= waitforstart;
        end
        else begin
            nextstate <= loadasset;
        end
    end
    waitforstart:
    begin
        if(irpowercount != irpowercountold)begin
            nextstate <= ingame;
        end
        else 
        begin
            nextstate <= waitforstart;
        end
    end
    ingame:
    begin
        if(collition == 1'b1)begin
            nextstate <= gameover;
        end
        else
            nextstate <= ingame; //! check if collition then gameover ?
    end
    gameover:
    begin
        if(irpowercount != irpowercountold)begin
            nextstate <= waitforstart;
        end
        else
            nextstate <= gameover;
    end
    //! need to add game state ltr
    endcase
end
// reset power button stuff lol 
always @(posedge clk or negedge rst)begin
    if(!rst)begin
        irpowercountold <= 0;
    end
    else begin
        irpowercountold <= irpowercount;
    end
end
always @(posedge clk or negedge rst) begin
    if(!rst)begin
        writeSRAMaddr <= 0; // this should be 0
        assetloadaddr <= 23'h47FFF + 1'b1; // change to the address of the asset
        SRAM_WE_N <= 1'b1; // write disable
        SRAM_CE_N <= 1'b1; // chip disable main control
        SRAM_OE_N <= 1'b0; // output disable when read need 0
        SRAM_DQ <= 16'bzzzzzzzzzzzzzzzz; // data bus
        sramwritestate <= setupSRAM;
    end
    else begin
        case(currentstate)
        loadasset:begin
            case (sramwritestate)
                setupSRAM:
                    begin
                        if(assetloadaddr == endofassetaddr + 1)begin
                            // set SRAM to readonly
                            sramwritestate <= setupSRAM;
                        end
                        else begin
                            SRAM_CE_N <= 1'b1; // chip enable
                            SRAM_WE_N <= 1'b0; // write enable
                            sramwritestate <= setSRaddr;
                        end
                    end
                setSRaddr:
                begin
                    // change address to write
                    sramwritestate <= setSRdata;
                end
                setSRdata:
                begin
                    // write data to SRAM
                    SRAM_CE_N <= 1'b0; // chip enable
                    SRAM_DQ <= {{8{1'b0}},FL_DQ};
                    sramwritestate <= setSRdone;
                end
                setSRdone:
                begin
                    // increment the address
                    SRAM_CE_N <= 1'b1; // chip disable
                    SRAM_WE_N <= 1'b1; // write disable
                    assetloadaddr <= assetloadaddr + 1'b1;
                    writeSRAMaddr <= writeSRAMaddr + 1'b1;
                    sramwritestate <= setupSRAM;
                end
            endcase
        end
        default:begin
            // setup for read cause we are not in loadasset state
            SRAM_DQ <= 16'bzzzzzzzzzzzzzzzz; //to read SRAM in other state
            SRAM_OE_N <= 1'b0; // output enable
            SRAM_CE_N <= 1'b0; // chip disable
            SRAM_WE_N <= 1'b1; // write disable
        end 
        endcase
    end
end
always@(posedge clk or negedge rst)begin
    if(!rst)begin
        currentstate <= loadasset;
    end
    else begin
        currentstate <= nextstate;
    end
end
wire [7:0] SRAM_LB;
assign SRAM_LB = SRAM_DQ[7:0];
always@(*)begin // for checking if next colour is background or stuff
    if (X > Xaxisstart + 288-1 || X <= Xaxisstart)
    begin
        nextcolor = 0; // black color
    end
    else if(Y > 400-1)
    begin
        nextcolor = 0; // black color
        if(Y >= 400)begin
            // display the base
            nextcolor = ROMBASE_OUT;
        end
    end
    else if(((counterVS >= V_BLANK && Y < (28+pipeylocation[0]))|| (Y >= (28+pipeylocation[0]+pipegap))) && counterHS >= H_BLANK && X >= (Xaxisstart + pipexlocation[0]) && X < (Xaxisstart + pipexlocation[0]+ romGREENPIPEwidth) && ROMGREENPIPE_OUT != 179 )begin
        // draw pipe here cause it has higher priority thab background but lower than bird and base
        if(firetagged[0])begin
            // check y if in range of fire if yes then blank it
            // check where the fire hits top or bottom pipe 
            if(shootfireylocationold[0] -35 <= pipeylocation[0]+28 && Y >= shootfireylocationold[0] - 35 && Y<= pipeylocation[0]+28)begin
                nextcolor = FL_DQ;
            end 
            else if(shootfireylocationold[0] + 35 >= pipeylocation[0]+28+pipegap && Y >= pipeylocation[0]+28+pipegap && Y < shootfireylocationold[0] + 35)begin
                nextcolor = FL_DQ;
            end
            else begin
                nextcolor = ROMGREENPIPE_OUT;
            end
        end
        else 
            nextcolor = ROMGREENPIPE_OUT;
    end
    else if(((counterVS >= V_BLANK && Y < (28+pipeylocation[1]))|| (Y >= (28+pipeylocation[1]+pipegap))) && counterHS >= H_BLANK && X >= (Xaxisstart + pipexlocation[1]) && X < (Xaxisstart + pipexlocation[1]+ romGREENPIPEwidth) && ROMGREENPIPE_OUT != 179 )begin
        if(firetagged[1])begin
            // check y if in range of fire if yes then blank it
            if(shootfireylocationold[1] -35 <= pipeylocation[1]+28 && Y >= shootfireylocationold[1] - 35 && Y<= pipeylocation[1]+28)begin
                nextcolor = FL_DQ;
            end 
            else if(shootfireylocationold[1] + 35 >= pipeylocation[1]+28+pipegap && // should remove until gap
                    Y >= pipeylocation[1]+28+pipegap && Y < shootfireylocationold[1] + 35)begin
                nextcolor = FL_DQ;
            end
            else begin
                nextcolor = ROMGREENPIPE_OUT;
            end
        end
        else 
            nextcolor = ROMGREENPIPE_OUT;
    end
    else
    begin
        nextcolor = FL_DQ; // background stuff
    end
    //? shoot fire
    if(firecounter != 0)begin
       if(X >= Xaxisstart + 144+17 && X < Xaxisstart + 288 && Y >= shootfireylocation - 35 && Y < shootfireylocation + 35 && ROMFLAME_OUT != 179)
         begin
            nextcolor = ROMFLAME_OUT;
         end 
    end
    //? end of shoot fire
    //highest priority always should be the bird
    // bird should be in midpoint of the screen
    // 179 is transparent color
    //         romYBIRDwidth/2
    if(X >= Xaxisstart + 144-17 && X < Xaxisstart + 144+17 && Y >= birdlocationY && counterVS >= V_BLANK && Y < birdlocationY + 24 && ROMYBIRD_OUT != 179)begin
        nextcolor = ROMYBIRD_map[ROMYBIRD_OUT];
        //! change color 
    end

    //get ready screen
    // 288/2 = 144                  400/2 = 200
    // 184/2 = 92                   267/2 = 133
    // so middle is 144-92 = 52     200-133 = 67
    if(currentstate == waitforstart && X >= Xaxisstart + 52 && X < Xaxisstart + 52 + getreadywidth && Y >= 67 && Y < 67 + getreadyheight && SRAM_LB != 179)begin
        nextcolor = SRAM_LB;
    end
    //? display score.
    if(currentstate == ingame && Y >= romNUMBERSheightoffset && Y < romNUMBERSheightoffset + romNUMBERSheight && ROMNUMBERS_OUT != 179)begin
        if(scorecount < 10)begin
            if(X >= Xaxisstart + 144 - (romNUMBERSwidth/2) && X < Xaxisstart + 144 + (romNUMBERSwidth/2))begin
                nextcolor = ROMNUMBERS_OUT;
            end
        end
        else begin
            // only 0 - 99 is done
            if(X>= Xaxisstart + 144 - (romNUMBERSbetweennumberGAP/2)-romNUMBERSwidth && X < Xaxisstart + 144 - (romNUMBERSbetweennumberGAP/2))begin
                nextcolor = ROMNUMBERS_OUT;
            end
            else if(X>= Xaxisstart + 144 + (romNUMBERSbetweennumberGAP/2) && X < Xaxisstart + 144 + (romNUMBERSbetweennumberGAP/2)+romNUMBERSwidth)begin
                nextcolor = ROMNUMBERS_OUT;
            end
        end
    end
    if(SW[1]&&(Y==birdlocationY|| Y == birdlocationY + 24||Y == pipeylocation[0]+28 || Y == pipeylocation[0]+pipegap+28 || X == pipexlocation[0]+144+(romGREENPIPEwidth/2)+5 || X == pipexlocation[0]+romGREENPIPEwidth+144+(romGREENPIPEwidth/2)+5 || X== Xaxisstart+144-17 || X== Xaxisstart+144+17))begin
        nextcolor = 0;
    end
    if(currentstate == gameover)begin
        //? scoreboard
        if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2) && X < Xaxisstart+144+(romSCOREBOARDwidth/2) && SCOREBOARD_OUT != 179)begin
            // start to end of scorebaord X axis 
            if(Y >= scoreboardyloc && Y < scoreboardyloc + romSCOREBOARDheight)begin
                nextcolor = SCOREBOARD_OUT;
            end
        end
        //? medal point is X = 26 and Y = 42
        //Xaxisstart+144-(romSCOREBOARDwidth/2) start X location
        //scoreboadyloc is the start Y location
        if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+26 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+26+romMEDALwidth && ROMMEDAL_OUT != 179 && medaltype != 0)begin
            if(Y >= scoreboardyloc+42 && Y < scoreboardyloc+42+romMEDALheight)begin
                nextcolor = ROMMEDAL_OUT;
            end
        end
        // all x y is based on the scoreboard
        //              2   1   0
        //reset addr at165 178 191 
        //small numberx166 179 192
        //small numbery034 034 034
        //0
        if(Y >= scoreboardyloc+34 && Y < scoreboardyloc+34+romSMALLNUMBERheight && ROMSMALLNUMBER_OUT != 179)begin
            if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+192 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+192+romSMALLNUMBERwidth && scorecountsmall[0] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
            else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+179 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+179+romSMALLNUMBERwidth&& scorecountsmall[1] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
            else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+166 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+166+romSMALLNUMBERwidth&& scorecountsmall[2] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
        end
        if(Y >= scoreboardyloc+76 && Y < scoreboardyloc+76+romSMALLNUMBERheight && ROMSMALLNUMBER_OUT != 179)begin
            if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+192 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+192+romSMALLNUMBERwidth && scorecountsmallbest[0] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
            else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+179 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+179+romSMALLNUMBERwidth&& scorecountsmallbest[1] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
            else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+166 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+166+romSMALLNUMBERwidth&& scorecountsmallbest[2] != 10)begin
                nextcolor = ROMSMALLNUMBER_OUT;
            end
        end
    end
    // debug scoring stuff 
    if(SW[1])begin
        if(firetagged[0])begin
            //if(Y == shootfireylocationold[0] + 35) // btm fire
            //    nextcolor = 179;
            if(Y == birdlocationY + romYBIRDheight) // bird bottom
                nextcolor = 179;
            //if(Y == pipeylocation[0]+pipegap)begin // btm pipe collition but it included bird height
            //    nextcolor = 179;
            //end
            //if(X== 144-romYBIRDwidth-romGREENPIPEwidth) // debug reset collition
            //    nextcolor = 179;
            if(Y == pipeylocation[0]+28)
                nextcolor = 179;
            
        end
        if(firetagged[1])begin

        end 
    end
    if(SW[1] && (X == Xaxisstart + 144 || pipexlocation[0]+romGREENPIPEwidth+144+5 == X))begin
        nextcolor = 255;
    end
    // debug score eh
    //if(SW[1] && (X == Xaxisstart+144-(romSCOREBOARDwidth/2)+191 || Y == scoreboardyloc+34))begin
    //    nextcolor = 110;
    //end
    if(X> Xaxisstart + 288 && X < Xaxisstart + 288 + FlameBarwidth- FlameBarExtrawidth && Y>= 1 &&Y <= FlameBarheight + 1 && currentstate == ingame)begin
        if(scorecount - shootfirepointold == 0)begin
            // nothing
        end
        else if (scorecount - shootfirepointold == 1)begin
            // half bar
            if(X < Xaxisstart + 288 + 1 + (FlameBarwidth-FlameBarExtrawidth)/2)begin
                if(ROMFLAMEREADYBAR_OUT == 0)
                    nextcolor = 115;
                else
                    nextcolor = 119;
            end
        end
        else begin
            // full bar
            if(ROMFLAMEREADYBAR_OUT == 0)
                nextcolor = 115;
            else
                nextcolor = 119;
        end
    end
    if(X> Xaxisstart + 288 -1 && X < Xaxisstart + 288 + FlameReadywidth && Y <= FlameReadyheight && ROMFLAMEREADY_OUT != 0 && currentstate == ingame)begin
        if(Y >= FlameReadyCutoffy && canshootfire)begin
            nextcolor = 117; // Red color
        end
        else if(Y<= FlameReadyCutoffy)
            nextcolor = 117;
    end
end

//? draw addr counter
always@(posedge clk25M)
begin
	if (!rst) 
	begin
        counter <= 4;
        selectcolor <= 100;
        ROMBASE_ADDR <= 0;
        ROMYBIRD_ADDR <= 0;
        ROMNUMBERS_ADDR <= 0;
   end
	else 
	begin
        //? Flame Ready ROM 
        if(currentstate == ingame)begin
        if(X == Xaxisstart + 288 - 1 && Y <= FlameReadyheight)begin
            ROMFLAMEREADY_ADDR <= Y*FlameReadywidth; // reset the addr
        end
        else if (X> Xaxisstart + 288 -1 && X < Xaxisstart + 288 + FlameReadywidth && Y <= FlameReadyheight)begin
            ROMFLAMEREADY_ADDR <= ROMFLAMEREADY_ADDR + 1'b1;
        end
        else begin
            ROMFLAMEREADY_ADDR <= ROMFLAMEREADY_ADDR;
        end
        if(X == Xaxisstart + 288 && Y <= FlameBarheight + 1)
            ROMFLAMEREADYBAR_ADDR <= (FlameBarwidth-FlameBarExtrawidth)*(Y-1) + flamebarrunning;
        else if(X> Xaxisstart + 288 && X < Xaxisstart + 288 + FlameBarwidth- FlameBarExtrawidth && Y>= 1 &&Y <= FlameBarheight + 1)begin
            ROMFLAMEREADYBAR_ADDR <= ROMFLAMEREADYBAR_ADDR + 1'b1;
        end
        end
        else begin
            ROMFLAMEREADY_ADDR <= 0;
            ROMFLAMEREADYBAR_ADDR <= 0;
        end

        //? End of flame ready ROM
        // X and Y are the coordinates of the pixel
        //only when width is < 184 and height is < 267 counter is incremented if X and Y are 480 and 640 reset counter
        if(currentstate == waitforstart && X >= Xaxisstart + 52 && X < Xaxisstart + 52 + getreadywidth && Y >= 67 && Y < 67 + getreadyheight)begin
            readSRAMaddr <= readSRAMaddr + 1'b1;
        end
        else if(X == 0 && Y == 0)begin
            readSRAMaddr <= 0;
        end
        else if(currentstate != waitforstart)begin
            readSRAMaddr <= 0;
        end
        else begin
            readSRAMaddr <= readSRAMaddr;
        end
        physicsclk <= 1'b0; // always 0 except when X and Y are 480 and 640 to update the physics
        // pipe address control.
        //?if(X == 0 && Y == 0) // reset the pipe address
        //?begin
        //?    ROMGREENPIPE_ADDR <= ((28+pipeylocation)*romGREENPIPEwidth);
        //?end
        //?else if (Y == 28+pipeylocation+10) // inside the pipe gap
        //?begin
        //?    ROMGREENPIPE_ADDR <= 0;
        //?end //if(((counterVS >= V_BLANK && Y < (28+pipeylocation))|| (Y >= (28+pipeylocation+pipegap))) && counterHS >= H_BLANK && X >= pipexlocation && X <= pipexlocation+ romGREENPIPEwidth)
        //?else if (counterVS >= V_BLANK && Y < (28+pipeylocation) && counterHS >= H_BLANK && X >= Xaxisstart + pipexlocation && X < Xaxisstart+ pipexlocation+ romGREENPIPEwidth)
        //?begin
        //?    ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR - 1'b1;
        //?end
        //?//if((counterVS >= V_BLANK && Y < (28+pipeylocation)) && counterHS >= H_BLANK && X >= pipexlocation && X <= pipexlocation+ romGREENPIPEwidth)
        //?else if (counterVS >= V_BLANK && Y >= (28+pipeylocation+pipegap) && counterHS >= H_BLANK && X >= Xaxisstart+pipexlocation && X < Xaxisstart+ pipexlocation+ romGREENPIPEwidth)
        //?begin
        //?    ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR + 1'b1;
        //?end
        //?else begin
        //?    ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR;
        //?end
        ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR;
        if(X == Xaxisstart + pipexlocation[0] - 1 && Y >= 0 && Y <= pipeylocation[0]+28) // reset the pipe address
        begin
            ROMGREENPIPE_ADDR <= ((28+pipeylocation[0])*romGREENPIPEwidth)-(romGREENPIPEwidth*Y);
        end
        else if (Y >= 28+pipeylocation[0]+pipegap && X == Xaxisstart + pipexlocation[0] - 1) // inside the pipe gap
        begin
            ROMGREENPIPE_ADDR <= romGREENPIPEwidth*(Y-(28+pipeylocation[0]+pipegap));
        end //if(((counterVS >= V_BLANK && Y < (28+pipeylocation[0]))|| (Y >= (28+pipeylocation[0]+pipegap))) && counterHS >= H_BLANK && X >= pipexlocation[0] && X <= pipexlocation[0]+ romGREENPIPEwidth)
        else if (counterVS >= V_BLANK && Y < (28+pipeylocation[0]) && counterHS >= H_BLANK && X >= Xaxisstart + pipexlocation[0] && X < Xaxisstart+ pipexlocation[0]+ romGREENPIPEwidth)
        begin
            ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR - 1'b1;
        end
        //if((counterVS >= V_BLANK && Y < (28+pipeylocation[0])) && counterHS >= H_BLANK && X >= pipexlocation[0] && X <= pipexlocation[0]+ romGREENPIPEwidth)
        else if (counterVS >= V_BLANK && Y >= (28+pipeylocation[0]+pipegap) && counterHS >= H_BLANK && X >= Xaxisstart+pipexlocation[0] && X < Xaxisstart+ pipexlocation[0]+ romGREENPIPEwidth)
        begin
            ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR + 1'b1;
        end

        if(X == Xaxisstart + pipexlocation[1] - 1 && Y >= 0 && Y <= pipeylocation[1]+28) // reset the pipe address
        begin
            ROMGREENPIPE_ADDR <= ((28+pipeylocation[1])*romGREENPIPEwidth)-(romGREENPIPEwidth*Y);
        end
        else if (Y >= 28+pipeylocation[1]+pipegap && X == Xaxisstart + pipexlocation[1] - 1) // inside the pipe gap
        begin
            ROMGREENPIPE_ADDR <= romGREENPIPEwidth*(Y-(28+pipeylocation[1]+pipegap));
        end //if(((counterVS >= V_BLANK && Y < (28+pipeylocation[1]))|| (Y >= (28+pipeylocation[1]+pipegap))) && counterHS >= H_BLANK && X >= pipexlocation[1] && X <= pipexlocation[1]+ romGREENPIPEwidth)
        else if (counterVS >= V_BLANK && Y < (28+pipeylocation[1]) && counterHS >= H_BLANK && X >= Xaxisstart + pipexlocation[1] && X < Xaxisstart+ pipexlocation[1]+ romGREENPIPEwidth)
        begin
            ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR - 1'b1;
        end
        //if((counterVS >= V_BLANK && Y < (28+pipeylocation[1])) && counterHS >= H_BLANK && X >= pipexlocation[1] && X <= pipexlocation[1]+ romGREENPIPEwidth)
        else if (counterVS >= V_BLANK && Y >= (28+pipeylocation[1]+pipegap) && counterHS >= H_BLANK && X >= Xaxisstart+pipexlocation[1] && X < Xaxisstart+ pipexlocation[1]+ romGREENPIPEwidth)
        begin
            ROMGREENPIPE_ADDR <= ROMGREENPIPE_ADDR + 1'b1;
        end

        if (X == 639 && Y == 479)
        begin
            counter <= nightmodereg ? 147456 : 1'b0; // first background if night need add offset
            physicsclk <= 1'b1;
        end
        else if (counterHS >= H_BLANK && X >= Xaxisstart -1 && X < Xaxisstart + 288-1 && counterVS >= V_BLANK && Y < 400) // max X is 479 and max Y is 287
        begin
            counter <= counter + 1'b1;
        end
        else
        begin
            counter <= counter;
        end
        //? for rom base pointer
        //? remember if scanning then must be didnt reset properly
        if(counterHS >= H_BLANK && X > Xaxisstart -2 && X < Xaxisstart + romBASEwidth-1&& Y >= 400 && Y < 480-1)begin// 400 is when the base should be.
            ROMBASE_ADDR <= ROMBASE_ADDR + 1'b1;
        end
        else if(X == Xaxisstart + romBASEwidth && Y >= 400 && Y < 480-1)begin
            // change row cause actual length is 366 and not 288 so we need to add 48 to the base address
            ROMBASE_ADDR <= ROMBASE_ADDR + 48;
        end
        else begin
            ROMBASE_ADDR <= ROMBASE_ADDR;
            if(X == 0 && Y == 0)begin
                ROMBASE_ADDR <= baseX;
            end
        end
        //? for rom flame pointer
        if(firecounter != 0)begin
           if(X >= Xaxisstart + 144+17 && X < Xaxisstart + 288 && Y >= shootfireylocation - 35 && Y < shootfireylocation + 35)
            begin
                ROMFLAME_ADDR <= ROMFLAME_ADDR + 1'b1;
            end
            else if(X == Xaxisstart + 144+17-1 && Y == shootfireylocation - 35)begin
                ROMFLAME_ADDR <= 0;
            end
            else begin
                ROMFLAME_ADDR <= ROMFLAME_ADDR;
            end
        end
        //? here we go again 
        //? for numbers counter
        ROMNUMBERS_ADDR <= ROMNUMBERS_ADDR;
        if(currentstate == ingame && Y >= romNUMBERSheightoffset && Y < romNUMBERSheightoffset + romNUMBERSheight)begin
            if(scorecount < 10)begin
                if(X >= Xaxisstart + 144 - (romNUMBERSwidth/2) && X < Xaxisstart + 144 + (romNUMBERSwidth/2))begin
                    ROMNUMBERS_ADDR <= ROMNUMBERS_ADDR + 1'b1;
                end
            end
            else begin
                // only 0 - 99 is done
                if(X>= Xaxisstart + 144 - (romNUMBERSbetweennumberGAP/2)-romNUMBERSwidth && X < Xaxisstart + 144 - (romNUMBERSbetweennumberGAP/2))begin
                    ROMNUMBERS_ADDR <= ROMNUMBERS_ADDR + 1'b1;
                end
                else if(X>= Xaxisstart + 144 + (romNUMBERSbetweennumberGAP/2) && X < Xaxisstart + 144 + (romNUMBERSbetweennumberGAP/2)+romNUMBERSwidth)begin
                    ROMNUMBERS_ADDR <= ROMNUMBERS_ADDR + 1'b1;
                end
            end
        end
        else begin
            if(X==1 && Y == 1)begin
                if(scorecount < 10)begin
                    ROMNUMBERS_ADDR <= scorebcdlow*romNUMBERSoffset;
                end
                else begin
                    ROMNUMBERS_ADDR <= scorebcdhigh*romNUMBERSoffset;
                end
            end
        end
        if(X == Xaxisstart + 20 && Y <= romNUMBERSheightoffset+romNUMBERSheight+1)begin
            if(scorecount >= 10)begin
                ROMNUMBERS_ADDR <= scorebcdhigh*romNUMBERSoffset + (Y-romNUMBERSheightoffset)*romNUMBERSwidth;
            end
        end
        else if(X== Xaxisstart + 144)begin
            if(scorecount >= 10)begin
                ROMNUMBERS_ADDR <= scorebcdlow*romNUMBERSoffset + (Y-romNUMBERSheightoffset)*romNUMBERSwidth;
            end
        end

        //? for rom yellow bird pointer
        if(X == Xaxisstart+144-17-1 && Y == birdlocationY)begin
            ROMYBIRD_ADDR <= (birdflap == birdflapmid) ? romYBIRDmidoffset : (birdflap == birdflapup) ? romYBIRDupoffset : romYBIRDdownoffset;
        end else if (X >= Xaxisstart+144-17&& X < Xaxisstart+144+17 && Y >= birdlocationY && counterVS >= V_BLANK && Y < birdlocationY + 24)begin
            ROMYBIRD_ADDR <= ROMYBIRD_ADDR + 1'b1;
        end
        else begin
            ROMYBIRD_ADDR <= ROMYBIRD_ADDR;
        end
        selectcolor <= nextcolor;
        // if the bird is drawn finished then we need to reset the romYbird address

        //? for endscreen stuff.
        if(currentstate == gameover)begin
            //? scoreboard
            if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2) && X < Xaxisstart+144+(romSCOREBOARDwidth/2))begin
                // start to end of scorebaord X axis 
                if(Y >= scoreboardyloc && Y < scoreboardyloc + romSCOREBOARDheight)begin
                    SCOREBOARD_ADDR <= SCOREBOARD_ADDR + 1'b1;
                end
                else 
                    SCOREBOARD_ADDR <= SCOREBOARD_ADDR;
            end
            else
                SCOREBOARD_ADDR <= SCOREBOARD_ADDR;
            // reset the address when start of the scoreboard
            if(X <= Xaxisstart+144+(romSCOREBOARDwidth/2)-1 && Y == scoreboardyloc)begin
                SCOREBOARD_ADDR <= 0;
            end
            //? medal point is X = 26 and Y = 42
            //Xaxisstart+144-(romSCOREBOARDwidth/2) start X location
            //scoreboadyloc is the start Y location
            if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+26 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+26+romMEDALwidth)begin
                if(Y >= scoreboardyloc+42 && Y < scoreboardyloc+42+romMEDALheight)begin
                    ROMMEDAL_ADDR <= ROMMEDAL_ADDR + 1'b1;
                end
                else begin
                    ROMMEDAL_ADDR <= ROMMEDAL_ADDR;
                end
            end
            else begin
                ROMMEDAL_ADDR <= ROMMEDAL_ADDR;
            end
            if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+26-1 && Y == scoreboardyloc+42+romMEDALheight)begin
                if(medaltype != 0)
                    ROMMEDAL_ADDR <= (medaltype-1)*(romMEDALoffset);
            end
            // all x y is based on the scoreboard
            //              2   1   0
            //reset addr at165 178 191 
            //small numberx166 179 192
            //small numbery034 034 034
            //0
            if(Y >= scoreboardyloc+34 && Y < scoreboardyloc+34+romSMALLNUMBERheight)begin
                if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+192 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+192+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+179 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+179+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+166 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+166+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR;
                end
            end
            if(Y >= scoreboardyloc+76 && Y < scoreboardyloc+76+romSMALLNUMBERheight)begin
                if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+192 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+192+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+179 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+179+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else if(X >= Xaxisstart+144-(romSCOREBOARDwidth/2)+166 && X<Xaxisstart+144-(romSCOREBOARDwidth/2)+166+romSMALLNUMBERwidth)begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR + 1'b1;                
                end
                else begin
                    ROMSMALLNUMBER_ADDR <= ROMSMALLNUMBER_ADDR;
                end
            end
            //reset addr
            if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+191)begin
                ROMSMALLNUMBER_ADDR <= scorecountsmall[0]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+34))*romSMALLNUMBERwidth;
            end
            else if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+178)begin
                ROMSMALLNUMBER_ADDR <= scorecountsmall[1]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+34))*romSMALLNUMBERwidth;
            end
            else if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+165)begin
                ROMSMALLNUMBER_ADDR <= scorecountsmall[2]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+34))*romSMALLNUMBERwidth;
            end
            if(Y >= scoreboardyloc+76 && Y < scoreboardyloc+76+romSMALLNUMBERheight)begin
                if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+191)begin
                    ROMSMALLNUMBER_ADDR <= scorecountsmallbest[0]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+76))*romSMALLNUMBERwidth;
                end
                else if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+178)begin
                    ROMSMALLNUMBER_ADDR <= scorecountsmallbest[1]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+76))*romSMALLNUMBERwidth;
                end
                else if(X == Xaxisstart+144-(romSCOREBOARDwidth/2)+165)begin
                    ROMSMALLNUMBER_ADDR <= scorecountsmallbest[2]*romSMALLNUMBERoffset+(Y-(scoreboardyloc+76))*romSMALLNUMBERwidth;
                end
            end
        end
   end
end
// for moving the base
reg [7:0] baseX;
reg signed [8:0] verticalspeed;
parameter fallingconstant = 1;
parameter jumpconstant = 5;
reg irpressed;
assign LEDR[2] = irpressed;
reg signed [10:0] birdlocationY; // bird X will always be middle 288/2 = 144
reg [3:0] slowclkcounter;
reg [14:0] scoreboardyloc;
reg signed [13:0] pipexlocation[0:1];
reg [7:0] pipeylocation[0:1];
reg [7:0] scorecount; // 127 is the max score
reg [1:0] birdflap;
reg [3:0] delayflap;
reg [3:0] scorecountsmall[0:2]; // if dont display then 10
reg [3:0] scorecountsmallbest[0:2]; // if dont display then 10
parameter   birdflapmid = 1,
            birdflapup = 0,
            birdflapdown = 2;
wire [3:0] scorebcdhigh;
wire [3:0] scorebcdlow;
assign scorebcdhigh = (scorecount/10)%10;
assign scorebcdlow = scorecount%10; 
reg singletrigger;
reg isnewhighscore;
always @(posedge physicsclk,negedge rst)begin
    if(!rst)begin
        scoreboardyloc <= 480; // 640 is the max Y
        medaltype <= 0;
        singletrigger <=1;
        isnewhighscore <=1;
        updatescore <= 0;
    end
    else begin
        if(!KEY[1])begin
            // reset highscore
            highscoretoeeprom <= 0;
            updatescore <= 1;
        end
        else begin
            updatescore <= 0;
        end
        if(currentstate == gameover)begin
            if(singletrigger)begin
                singletrigger <= 0;
                // check eeprom highscore
                if(scorecount > highscorefromeeprom)begin
                    isnewhighscore <= 1;
                    highscoretoeeprom <= scorecount;
                    // write to eeprom
                    updatescore <= 1;
                end
            end
            if(scorecount > 39)begin
                medaltype <= medalplatinum;
            end
            else if(scorecount > 29)begin
                medaltype <= medalgold;
            end
            else if(scorecount > 19)begin
                medaltype <= medalsilver;
            end
            else if(scorecount > 9)begin
                medaltype <= medalbronze;
            end
            else begin
                medaltype <= medalnone;
            end
            if(scorecount > 9)begin
                if(scorecount > 99)begin
                    scorecountsmall[0] <= scorecount%10;
                    scorecountsmall[1] <= (scorecount/10)%10;
                    scorecountsmall[2] <= (scorecount/100)%10;
                end
                else begin
                    scorecountsmall[0] <= scorecount%10;
                    scorecountsmall[1] <= (scorecount/10)%10;
                    scorecountsmall[2] <= 10;
                end
            end
            else begin
                scorecountsmall[0] <= scorecount%10;
                scorecountsmall[1] <= 10;
                scorecountsmall[2] <= 10;
            end

            if(highscorefromeeprom > 9)begin
                if(highscorefromeeprom > 99)begin
                    scorecountsmallbest[0] <= highscorefromeeprom%10;
                    scorecountsmallbest[1] <= (highscorefromeeprom/10)%10;
                    scorecountsmallbest[2] <= (highscorefromeeprom/100)%10;
                end
                else begin
                    scorecountsmallbest[0] <= highscorefromeeprom%10;
                    scorecountsmallbest[1] <= (highscorefromeeprom/10)%10;
                    scorecountsmallbest[2] <= 10;
                end
            end
            else begin
                scorecountsmallbest[0] <= highscorefromeeprom%10;
                scorecountsmallbest[1] <= 10;
                scorecountsmallbest[2] <= 10;
            end

            // gradually move the scoreboard up
            if(scoreboardyloc > 400)begin
                scoreboardyloc <= scoreboardyloc - 3'd3;
            end
            else if (scoreboardyloc > 220)begin
                scoreboardyloc <= scoreboardyloc - 3'd2;
            end
            else if (scoreboardyloc > 180)begin
                scoreboardyloc <= scoreboardyloc - 3'd1;
            end
            else begin
                scoreboardyloc <= scoreboardyloc;
            end
        end
        else begin
            isnewhighscore<= 0;
            singletrigger <= 1;
            scoreboardyloc <= 480;
            scorecountsmall[0] <= 10;
            scorecountsmall[1] <= 10;
            scorecountsmall[2] <= 10;
            scorecountsmallbest[0] <= 10;
            scorecountsmallbest[1] <= 10;
            scorecountsmallbest[2] <= 10;
        end
    end
end
always @(posedge physicsclk,negedge rst)begin
    if(!rst)begin
        birdflap <= birdflapmid;
        delayflap <= 0;
    end
    else begin
        case(currentstate)
        waitforstart:begin
            birdflap <= birdflapmid;
            delayflap <= 0;
        end
        ingame:begin
            // loop the birdflap
            if(delayflap >= 4)begin
                delayflap <= 0;
                if(birdflap == birdflapup)begin
                    birdflap <= birdflapdown;
                end
                else begin
                    birdflap <= birdflap - 1'b1;
                end
            end
            else begin
                delayflap <= delayflap + 1'b1;
            end
        end
        default:begin
            birdflap <= birdflapmid;
            delayflap <= 0;
        end
        endcase
    end
end
reg firetagged[0:1];
reg [9:0]shootfireylocationold[0:1];
reg canshootfire;
reg [1:0]shootfirecdrcounter;
reg [9:0]shootfirepointold;
reg [3:0] flamebarrunning;
parameter shootfirecd = 2;
always @(posedge physicsclk, negedge rst)begin
    if(!rst)begin
        flamebarrunning <= 0;
    end
    else begin
        if(flamebarrunning >= FlameBarExtrawidth)begin
            flamebarrunning <= 0;
        end
        else 
            flamebarrunning <= flamebarrunning + 1'b1; 
    end
end
always @(posedge physicsclk, negedge rst)begin
    if(!rst)begin
        canshootfire <= 1'b0;
        shootfirecdrcounter <= 0;
        shootfirepointold <= 0;
    end
    else begin
        case(currentstate)
        ingame:begin
            if(scorecount - shootfirepointold >= shootfirecd)begin
                canshootfire <= 1'b1;
            end
            else begin
                canshootfire <= 1'b0;
            end
            if(canshootfire)begin
                if(irshoot != irshootold)begin
                    shootfirepointold <= scorecount;
                end
                else begin
                    shootfirepointold <= shootfirepointold;
                end
            end
        end
        default:begin
            canshootfire <= 0;
            shootfirecdrcounter <= 0;
            shootfirepointold <= 0;
        end
        endcase
    end
end
always @(posedge physicsclk,negedge rst)begin
    if(!rst)begin
        pipexlocation[0] <= 288;
        pipeylocation[0] <= 0;
        pipexlocation[1] <= 288;
        pipeylocation[1] <= 0;
        shootfireylocationold[0] <= 480;
        shootfireylocationold[1] <= 480;
        firetagged[0] <= 0;
        firetagged[1] <= 0;
        enemyenable[0] <= 0;
        enemyenable[1] <= 0;
    end
    else begin
        case(currentstate)
        waitforstart:begin
            pipexlocation[0] <= 288;
            pipeylocation[0] <= q[8:0]%206; // 206 is the max height of the pipe
            pipexlocation[1] <= 288;
            pipeylocation[1] <= q[8:0]%206; // 206 is the max height of the pipe
            firetagged[0] <= 0;
            firetagged[1] <= 0;
        end
        ingame:begin
        if(firecounter == shootfireduration)begin
            //check which pipe is > bird location if yes then fire the tag
            if(pipexlocation[0]+romGREENPIPEwidth+144+5 > Xaxisstart + 144)begin
                firetagged[0] <= 1'b1;
                shootfireylocationold[0] <= shootfireylocation;
            end
            if(pipexlocation[1]+romGREENPIPEwidth+144+5> Xaxisstart + 144 && pipexlocation[1] != 288)begin
                firetagged[1] <= 1'b1;
                shootfireylocationold[1] <= shootfireylocation;
            end
        end// fireshots
        if(pipexlocation[0] == -romGREENPIPEwidth)begin
            firetagged[0] <= 1'b0;
            pipexlocation[0] <= 288;
            pipeylocation[0] <= q[8:0]%206;
        end
        else begin
            pipexlocation[0] <= pipexlocation[0] - 1'b1;
        end
        if(pipexlocation[1] != 288)begin
            if(pipexlocation[1] == -romGREENPIPEwidth)begin
                firetagged[1] <= 0;
                pipexlocation[1] <= 288;
                pipeylocation[1] <= q[8:0]%206;
            end
            else begin
                pipexlocation[1] <= pipexlocation[1] - 1'b1;
            end
        end
        if(SW[17] && pipexlocation[0] == 144-(romGREENPIPEwidth/2)) // in middle of the screen then spawn the second pipe
        begin
            pipexlocation[1] <= 287;
            pipeylocation[1] <= q[8:0]%206;
            if(scorecount - shootfirepointold >= shootfirecd && LFSR%3 == 2)begin
                // 1/3 chance to spawn the enemy
                enemyenable[1] <= 1'b1; // enable the enemy
            end
        end
        end
        default:begin
            enemyenable[0] <= 0;
            enemyenable[1] <= 0;
            pipexlocation[0] <= pipexlocation[0];
            pipeylocation[0] <= pipeylocation[0];
            pipexlocation[1] <= pipexlocation[1];
            pipeylocation[1] <= pipeylocation[1];
        end
        endcase
    end
end
reg [3:0]jumptriggered;
reg [0:0]enemyenable[0:1];
always @(posedge physicsclk,negedge rst) begin
    if(!rst)begin
        baseX <= 0;
        irpressed <= 1'b0;
        // width excluding the base is 400 so the bird should be in the middle of the screen 400/2 = 200
        birdlocationY <= 200;
        ircountold <= 0;
        verticalspeed <= 0;
        slowclkcounter <= 0;
        scorecount <= 0;
        jumptriggered <= 0;
    end
    else begin
        // need to add condition if to let the base move only when the bird is alive / moving.
        case(currentstate)
            waitforstart:begin
                // reset all stuff
                if (baseX == 48)begin
                    baseX <= 0;
                end
                else begin
                    baseX <= baseX + 1'b1;
                end
                ircountold <= ircount;// just to reset the ircount
                birdlocationY <= 200;
                verticalspeed <= 0;
                slowclkcounter <= 0;
                scorecount <= 0;
                collition <= 1'b0;
            end
            ingame:begin
                if (baseX == 48)begin
                    baseX <= 0;
                end
                else begin
                    baseX <= baseX + 1'b1;
                end
                irpressed <= 1'b0;
                if (ircount != ircountold)begin
                    jumptriggered <= jumptriggered + 1;
                    ircountold <= ircount;
                    irpressed <= 1'b1;
                    verticalspeed <= -jumpconstant;
                end
                else begin
                    slowclkcounter <= slowclkcounter + 1'b1;
                    if(slowclkcounter == 3)begin
                        slowclkcounter <= 0;
                        verticalspeed <= verticalspeed + fallingconstant; // 1/3 falling speed
                    end
                    if(birdlocationY + romYBIRDheight >= 400 && ircount == ircountold && verticalspeed > 0)begin
                        verticalspeed <= 0;
                    end
                end
                birdlocationY <= (birdlocationY + verticalspeed) > 0 ? (birdlocationY + verticalspeed) : 0;
                collition <= 1'b0;
                // calculate score
                scorecount <= scorecount;
                if(pipexlocation[0]+romGREENPIPEwidth+144+5 == Xaxisstart + 144)begin
                    scorecount <= scorecount + 1'b1;
                end
                if(pipexlocation[1]+romGREENPIPEwidth+144+5 == Xaxisstart + 144)begin
                    scorecount <= scorecount + 1'b1;
                end
                //// debug medal 
                ////scorecount <= SW[16:6];
                // detect collition with pipe
                // <=28+pipeylocation is the top of the pipe wall height 
                // >=28+pipeylocation+pipegap+ birdheight 
                // pipeylocation 
                // birdlocationY is the top of the bird
                // pipexlocation+144+(romGREENPIPEwidth/2)+5 >= Xaxisstart+144-17 && pipexlocation+romGREENPIPEwidth+144+(romGREENPIPEwidth/2)+5 <= Xaxisstart+144+17
                // 1 pipexlocation+144+(romGREENPIPEwidth/2)+5
                // 2 pipexlocation+romGREENPIPEwidth+144+(romGREENPIPEwidth/2)+5
                // 3 Xaxisstart+144-17
                // 4 Xaxisstart+144+17
                // 24 is the bird height
                if( !SW[2] &&( birdlocationY <= 28+pipeylocation[0]  || birdlocationY + 24 >= pipeylocation[0]+pipegap+28) && 
                pipexlocation[0]+144+(romGREENPIPEwidth/2)+5 <= Xaxisstart+144+17 && 
                pipexlocation[0]+romGREENPIPEwidth+144+(romGREENPIPEwidth/2)+5 >= Xaxisstart+144-17)begin
                    collition <= 1'b1;
                    birdlocationY <= birdlocationY;
                end
                if( !SW[2] &&( birdlocationY <= 28+pipeylocation[1]  || birdlocationY + 24 >= pipeylocation[1]+pipegap+28) && 
                pipexlocation[1]+144+(romGREENPIPEwidth/2)+5 <= Xaxisstart+144+17 && 
                pipexlocation[1]+romGREENPIPEwidth+144+(romGREENPIPEwidth/2)+5 >= Xaxisstart+144-17)begin
                    collition <= 1'b1;
                    birdlocationY <= birdlocationY;
                end
                if(firetagged[0] && pipexlocation[0] >= 144-romYBIRDwidth-romGREENPIPEwidth)begin
                // check y if in range of fire if yes then blank it
                // check where the fire hits top or bottom pipe 
                    if(shootfireylocationold[0] -35 <= pipeylocation[0]+28 && birdlocationY >= shootfireylocationold[0] - 35 && birdlocationY<= pipeylocation[0]+romYBIRDheight+28)begin
                        collition <= 1'b0;
                        birdlocationY <= (birdlocationY + verticalspeed) > 0 ? (birdlocationY + verticalspeed) : 0;
                    end 
                    else if(shootfireylocationold[0] + 35 >= pipeylocation[0]+pipegap && birdlocationY >= pipeylocation[0]+pipegap && birdlocationY + romYBIRDheight < shootfireylocationold[0] + 35)begin
                        collition <= 1'b0;
                        birdlocationY <= (birdlocationY + verticalspeed) > 0 ? (birdlocationY + verticalspeed) : 0;
                    end
                end
                if(firetagged[1] && pipexlocation[1] >= 144-romYBIRDwidth-romGREENPIPEwidth)begin
                // check y if in range of fire if yes then blank it
                // check where the fire hits top or bottom pipe 
                    //            determine if pipe on top                                  
                    if(shootfireylocationold[1] -35 <= pipeylocation[1]+28 && birdlocationY >= shootfireylocationold[1] - 35 && birdlocationY<= pipeylocation[1]+romYBIRDheight+28)begin
                        collition <= 1'b0;
                        birdlocationY <= (birdlocationY + verticalspeed) > 0 ? (birdlocationY + verticalspeed) : 0;
                    end 
                    else if(shootfireylocationold[1] + 35 >= pipeylocation[1]+pipegap && birdlocationY >= pipeylocation[1]+pipegap && birdlocationY + romYBIRDheight < shootfireylocationold[1] + 35)begin
                        collition <= 1'b0;
                        birdlocationY <= (birdlocationY + verticalspeed) > 0 ? (birdlocationY + verticalspeed) : 0;
                    end
                end
                // this must be the highest priority
                if(birdlocationY + romYBIRDheight >= 400)begin
                    birdlocationY <= birdlocationY;
                    if(verticalspeed < 0)begin //! this function is for debug only 
                        birdlocationY <= birdlocationY + verticalspeed;
                    end
                    //collition <= 1'b1; disable this to debug better
                end
            end
            gameover:
            begin
                scorecount <= scorecount;
                ircountold <= ircount; // just to reset the ircount
                baseX <= baseX;
                birdlocationY <= birdlocationY;
                verticalspeed <= 0;
                collition <= 1'b1;
            end
            default:begin
                ircountold <= ircount; // just to reset the ircount
                baseX <= baseX;
                birdlocationY <= birdlocationY;
                verticalspeed <= 0;
                collition <= 1'b0;
            end
        endcase
        // night mode switch
        if(enablenightmode == 1'b1)begin
            nightmodereg <= 1'b1;
        end
        else begin
            nightmodereg <= 1'b0;
        end
    end
end

reg [9:0] shootfireylocation;
reg [7:0] irshootold;
reg [15:0] firecounter;
parameter shootfireduration = 20;
always @(posedge physicsclk,negedge rst)begin
    if(!rst)begin
        shootfireylocation <= 0;
        irshootold <= 0;
        firecounter <= 0;
    end
    else begin
        case(currentstate)
        ingame:begin
            if(irshoot != irshootold && canshootfire)begin
                irshootold <= irshoot;
                if(birdlocationY > (romYBIRDheight/2)+35)
                    shootfireylocation <= birdlocationY+(romYBIRDheight/2);
                else 
                    shootfireylocation <= 35;
                firecounter <= shootfireduration;
            end
            if(firecounter != 0)begin
                firecounter <= firecounter - 1'b1;
            end
        end
        default:begin
            shootfireylocation <= 255;
            firecounter <= 0;
            irshootold <= irshoot;
        end
        endcase
    end
end

//? IR STUFF
wire            IR_READY;
wire    [31:0]  IR_DATA;
IR_RECEIVE IR(  .iCLK(clk), .iRST_n(rst),
                .iIRDA(IRDA_RXD), .oDATA_READY(IR_READY), .oDATA(IR_DATA));
parameter   IRA = 8'h0F,    IRB = 8'h13,    IRC = 8'h10,        IRPOWER = 8'h12,
            IR1 = 8'h01,    IR2 = 8'h02,    IR3 = 8'h03,        IRCUP = 8'h1A,
            IR4 = 8'h04,    IR5 = 8'h05,    IR6 = 8'h06,        IRCDOWN = 8'h1E,
            IR7 = 8'h07,    IR8 = 8'h08,    IR9 = 8'h09,        IRVUP = 8'h1B,
            IRMENU = 8'h11, IR0 = 8'h00,    IRRETURN = 8'h17,   IRVDOWN = 8'h1F,
            IRPLAY = 8'h16, IRLEFT = 8'h14, IRRIGHT = 8'h18,    IRMUTE = 8'h0C;
reg [7:0]ircount;
reg [7:0]ircountold;
reg [7:0]irpowercount;
reg [7:0]irpowercountold;
reg [7:0]irshoot;
always @(negedge IR_READY or negedge rst)begin
    if (!rst)begin
        ircount <= 0;
        irpowercount <= 0;
        selectaudioControl <=0;
        BGMvolume <= 6;
        SFXvolume <= 7;
        irshoot <= 0;
    end
    else begin
        // not checking for any button pressed
        case(IR_DATA[23:16])
            IR2:begin
                ircount <= ircount + 1'b1;
            end
            IR3:begin
                if(canshootfire)
                    irshoot <= irshoot + 1'b1;
                else 
                    irshoot <= irshoot;
            end
            IRPOWER:begin
                irpowercount <= irpowercount + 1'b1;
            end
            IRVUP:begin
                // check if selectAudioControl
                if(selectaudioControl == audioControlBGM)begin
                    if(BGMvolume < 7)begin
                        BGMvolume <= BGMvolume + 1'b1;
                    end
                    else begin
                        BGMvolume <= BGMvolume;
                    end
                end
                else begin
                    if(SFXvolume < 7)begin
                        SFXvolume <= SFXvolume + 1'b1;
                    end
                    else begin
                        SFXvolume <= SFXvolume;
                    end
                end
            end
            IRVDOWN:begin
                // check if selectAudioControl
                if(selectaudioControl == audioControlBGM)begin
                    if(BGMvolume > 0)begin
                        BGMvolume <= BGMvolume - 1'b1;
                    end
                    else begin
                        BGMvolume <= BGMvolume;
                    end
                end
                else begin
                    if(SFXvolume > 0)begin
                        SFXvolume <= SFXvolume - 1'b1;
                    end
                    else begin
                        SFXvolume <= SFXvolume;
                    end
                end
            end
            IRLEFT:begin
                selectaudioControl <= audioControlSFX;
            end
            IRRIGHT:begin
                selectaudioControl <= audioControlBGM;
            end
            default:begin
                ircount <= ircount;
                irpowercount <= irpowercount;
                irshoot <= irshoot;
            end
        endcase
    end
end
//* LFSR
reg [45:0]q ;
always @(posedge clk or negedge rst) begin
    if (!rst)
      q <= 8'd1; 
    else
      q <= {q[44:0], q[7] ^ q[5] ^ q[4] ^ q[3] ^ q[15] ^ q[11]^q[32]^q[40] ^q[23]}; // polynomial for maximal LFSR
end
//* SD CARD
sevenseg sevenseg4(debug[3:0],HEX0);
sevenseg sevenseg5(debug[7:4],HEX1);
sevenseg sevenseg6(debug[11:8],HEX2);
sevenseg sevenseg7(debug[15:12],HEX3);
//reg [17:0]countss;
//reg flag;
//always@(posedge AUD_DACLRCK or negedge rst)begin
//    if(!rst)begin
//        debug <= 0;
//        countss <= 0;
//        flag <= 0;
//    end
//    else begin
//        if(SD_AudioData != debug && !flag)begin
//            debug <= SD_AudioData;
//            countss <= countss + 1'b1;
//        end
//        if(countss == SW[17:0])begin
//            flag <= 1'b1;
//        end
//    end
//end
output SD_CLK;
inout SD_CMD;
inout [3:0]SD_DAT;
assign SD_DAT[0] = 1'bZ;
assign SD_DAT[1] = 1'b1;
assign SD_DAT[2] = 1'b1;
assign SD_DAT[3] = 1'b1;
wire [3:0] card_stat;
wire [1:0] card_type;
reg         rstart;
reg [31:0]  rsector;
wire         rbusy;
wire         rdone;
wire          outen;             // when outen=1, a byte of sector content is read out from outbyte
wire [ 8:0]  outaddr;           // outaddr from 0 to 511, because the sector size is 512
wire [ 7:0]  outbyte;            // a byte of sector content
sd_reader reader(
    .rstn(rst),
    .clk(clk),
    .sdclk(SD_CLK),
    .sdcmd(SD_CMD),
    .sddat0(SD_DAT[0]),
    .card_stat(card_stat),
    .card_type(card_type),
    .rstart(rstart),
    .rsector(rsector),
    .rbusy(rbusy),
    .rdone(rdone),
    .outen(outen),
    .outaddr(outaddr),
    .outbyte(outbyte)
);
wire signed [31:0] SD_AudioData;
wire readblank;
wire fulldata;
assign LEDR[0] = readblank;
assign LEDR[6] = fulldata;
wire [13:0] fifospace;
SDFIFO sdfifo(
    .aclr(!rst),
    .data(outbyte),
    .rdclk(audioclk2),
    .rdreq(fifospace > 512?audioROMreadenable:0),
    .wrclk(clk),
    .wrreq(outen),
    .q(SD_AudioData),
    .rdempty(readblank),
    .wrfull(fulldata),
    .wrusedw(fifospace)
);
parameter SDaudiosectorstart = 2;
parameter SDaudiosectorend = 3192;
parameter SDaudio2sectorstart = 3193;
parameter SDaudio2sectorend = 21299;
parameter SDsinestart = 21300;
parameter SDsineend = 22549;
always @(posedge clk or negedge rst) begin
    if(!rst)begin
        rstart <= 1'b0;
        rsector <= SDaudio2sectorstart;
    end else begin
        if(fifospace < 4098 && !rbusy)begin
            if(rsector >=SDaudio2sectorend)begin
                rsector <= SDaudio2sectorstart;
            end
            else
                rsector <= rsector + 1'b1;
            rstart <= 1'b1;
        end
        else begin
            if(rdone)
                rstart <= 1'b0;
        end
    end
end
//* END SD CARD
endmodule