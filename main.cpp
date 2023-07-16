#include "mbed.h"
#include "stdio.h"

BufferedSerial uart0(P0_25, P0_26,115200);  //TX, RX
SPI dds(P0_9,P0_8,P0_7);    //dds spi
DigitalOut le1(P0_6);       //dds1
DigitalOut le2(P0_5);       //dds2
DigitalOut ps(P0_4);       //dds2 phase select
SPI adc(P0_18,P0_17,P0_15);    //adc
DigitalOut cs(P0_20);       //adc cs
DigitalIn drdy(P0_19);   //adc drdy
DigitalOut cs1(P0_10);       //pll1 cs
DigitalOut cs2(P0_11);       //pll2 cs
DigitalIn ld1(P0_0);   //pll1 lock det.
DigitalIn ld2(P0_1);   //pll1 lock det.

DigitalOut att1(P2_1);    //att0. -10dB. active lo
DigitalOut att2(P2_2);    //att1. -20dB
DigitalOut sel(P2_0);   //on-brd output sel. P1-2(1) or P3-4(0)
DigitalOut ref_sw(P2_3);   //IN_SW. ref_in sw. P1-2(1) or P3-4(0)
DigitalOut sig_sw(P2_4);   //OUT_SE. sig_in sw. P1-2(1) or P3-4(0)

DigitalOut sel1(P1_18);     //P1-2 stim. sel. P1(1) or P2(0)
DigitalOut sel2(P1_20);     //P1-2 sig. sel. P1(1) or P2(0)
DigitalOut sel3(P1_19);     //P3-4 stim. sel. P3(1) or P4(0)
DigitalOut sel4(P1_21);     //P3-4 sig. sel. P3(1) or P4(0)

//uart control
const uint8_t buf_size=12;
char read_buf[buf_size];    //uart read buf
void buf_read(uint8_t num); //uart read func.
void buf2val();             //buf to vals change func. return to 'freq' and 'ampl' global var 
void val_send(float val);  //uart send func.

//DDS control
#define res_inv 4           //res=67108864/2^28
#define ref_freq 25000000   //PLL ref clk. Hz
#define dif_freq 400000     //dif. freq. 400kHz
void dds_set();    //dds set func.

//adc control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;
void drdy_wait();
int16_t adc_read(uint8_t ch);

//PLL ontrol
const uint8_t fpfd=25;
const uint16_t nmod=25000;  //fpfd/spacing(0.001MHz)
uint32_t freq;              //kHz
void pll_spi_send(uint8_t ch,uint32_t in);
void pll_set(uint8_t ch,uint32_t freq);
uint8_t k;
uint32_t data[6]={0,0,0x4E42,0x4B3,0,0x580005};
char local_buf[2];

//aux control
uint8_t att;
uint8_t port;
void att_set(uint8_t att);
void port_set(uint8_t port);

//calc.
uint8_t integ,i,j;
int16_t ref_i[2],sig_i[2];
float ref_f[2],sig_f[2],den,s_re,s_im;

int main(){
    le1=1;
    le2=1;
    ps=0;
    cs=1;
    cs1=1;
    cs2=1;
    dds.format(16,2);   //spi mode setting. 2byte(16bit) transfer, mode 2
    //adc init
    cs=0;
    adc.write(rst);
    cs=1;
    thread_sleep_for(10);
    cs=0;
    adc.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    adc.write((0b010<<5)+(0b10<<3)+(0b1<<2));       //180sps, turbo mode, cc mode
    cs=1;
    //dds init
    dds_set();
    while (true){
        buf_read(buf_size); //uart buf read
        buf2val();
        pll_set(1,freq);
        pll_set(2,freq+dif_freq/1000);
        att_set(att);
        port_set(port);    //S11(0) S12(1) S13(2) S14(3), S21(4) S22(5) S23(6) S24(7), S31(8) S32(9) S33(10) S34(11), S41(12) S42(13) S43(14) S44(15)
        for(i=0;i<2;++i){   //meas
            ref_f[i]=0;
            sig_f[i]=0;
            ps=i;
            thread_sleep_for(10);
            for(j=0;j<integ;++j){
                ref_i[i]=adc_read(0);
                ref_f[i]=ref_f[i]+(float)ref_i[i];
            }
            for(j=0;j<integ;++j){
                sig_i[i]=adc_read(1);
                sig_f[i]=sig_f[i]+(float)sig_i[i];
            }
            ref_f[i]=ref_f[i]/integ;
            sig_f[i]=sig_f[i]/integ;
        }
        
        //calc. and send
        den=ref_f[0]*ref_f[0]+ref_f[1]*ref_f[1];    //Re(re)^2+Im(re)^2
        s_re=(sig_f[0]*ref_f[0]+sig_f[1]*ref_f[1])/den;
        s_im=(sig_f[1]*ref_f[0]-sig_f[0]*ref_f[1])/den;
        val_send(s_re);
        val_send(s_im);
    }
}

//uart char number read func.
void buf_read(uint8_t num){
    char local_buf[1];
    uint8_t i;
    for (i=0;i<num;++i){
        uart0.read(local_buf,1);
        read_buf[i]=local_buf[0];
    }
}

//buf to val change func.
void buf2val(){
    uint8_t i,j;
    uint32_t pow10;
    freq=0;
    integ=0;
    att=0;
    port=0;
    for(i=0;i<7;++i){
        pow10=1;
        for(j=0;j<7-i;++j){
            pow10=10*pow10;
        }
        freq=freq+(read_buf[i]-48)*pow10;
    }
    for(i=0;i<2;++i){
        pow10=1;
        for(j=0;j<2-i;++j){
            pow10=10*pow10;
        }
        integ=integ+(read_buf[i+7]-48)*pow10;
    }
    att=read_buf[i+9]-48;
    for(i=0;i<2;++i){
        pow10=1;
        for(j=0;j<2-i;++j){
            pow10=10*pow10;
        }
        port=port+(read_buf[i+10]-48)*pow10;
    }
}

//uart send func.
void val_send(float val){
    char local_buf[1];
    char data[5];
    uint8_t i;
    uint64_t integer,frac;
    if(val<0){
        val=abs(val);
        local_buf[0]=45;
        uart0.write(local_buf,1);   //send minus
    }else{
        val=val;
        local_buf[0]=43;
        uart0.write(local_buf,1);   //send plus
    }
    integer=(uint64_t)(val);
    data[1]=0x30+(integer)%10;        //1
    data[0]=0x30+(integer/10)%10;       //10
    for(i=0;i<2;++i){
        local_buf[0]=data[i];
        uart0.write(local_buf,1);
    }

    local_buf[0]=46;
    uart0.write(local_buf,1);   //send '.'
    
    frac=(uint64_t)((val-integer)*100000);
    data[4]=0x30+frac%10;            //0.00001
    data[3]=0x30+(frac/10)%10;       //0.0001
    data[2]=0x30+(frac/100)%10;      //0.001
    data[1]=0x30+(frac/1000)%10;       //0.01
    data[0]=0x30+(frac/10000)%10;      //0.1
    for(i=0;i<5;++i){
        local_buf[0]=data[i];
        uart0.write(local_buf,1);
    }
}

//wave set func.
void dds_set(){
    uint16_t buf;   //spi send buf
    char set[2];    //i2c send buf
    uint16_t pha=0; //for adjust

    //dds1 pll clk
    le1=0;
    dds.write(0x2100);
    buf=((res_inv*ref_freq)&0x3FFF)+0x4000;
    dds.write(buf);
    buf=((res_inv*ref_freq)>>14)+0x4000;
    dds.write(buf);
    buf=(4096*pha/360)+0xC000;  //pha reg 0 = 0 deg, adjusted
    dds.write(buf);
    le1=1;
    
    //lock-in LO
    le2=0;
    dds.write(0x2100);
    buf=((res_inv*dif_freq)&0x3FFF)+0x4000;
    dds.write(buf);
    buf=((res_inv*dif_freq)>>14)+0x4000;
    dds.write(buf);
    buf=0+0xC000;  //pha reg 0 = 0 deg
    dds.write(buf);
    buf=(4096*90/360)+0xE000; //pha reg 1 = 90 deg
    dds.write(buf);
    le2=1;

    le1=0;
    le2=0;
    dds.write(0x2000+0x200);      //accum. reset, pi/sw=1, pin sel mode
    le1=1;
    le2=1;
}

void drdy_wait(){
    while(true){
        if(drdy==0) break;
    }
}

int16_t adc_read(uint8_t ch){
    uint8_t buf[2];     //spi receive buf
    adc.format(8,1);
    cs=0;
    adc.write((wreg<<4));       //write addr 0x00, 1byte
    if(ch==0)adc.write((0b0000<<4)+1);   //ch0 ch1 bipolar mux, pga disable h81
    if(ch==1)adc.write((0b0101<<4)+1);   //ch2 ch3 bipolar mux, pga disable h81
    cs=1;

    drdy_wait();
    cs=0;
    buf[1]=adc.write(0x00);
    buf[0]=adc.write(0x00);
    cs=1;
    return(buf[1]<<8)+buf[0];
}

void pll_spi_send(uint8_t ch,uint32_t in){
    uint8_t buf;
    if(ch==1)cs1=0;
    else cs2=0;
    buf=(in>>24)&0xff;
    adc.write(buf);
    buf=(in>>16)&0xff;
    adc.write(buf);
    buf=(in>>8)&0xff;
    adc.write(buf);
    buf=(in>>0)&0xff;
    adc.write(buf);
    if(ch==1)cs1=1;
    else cs2=1;
}

void pll_set(uint8_t ch,uint32_t freq){
    uint8_t i,j;
    uint8_t ndiv;
    uint16_t nint,nmodulus,nfrac_i,r,a,b;
    float freq_f,temp,nfrac;
    if(freq>4400000)freq=4400000;
    if(freq<35)freq=35;
    i=0;
    ndiv=2;
    freq_f=(float)freq;
    freq_f=freq_f/1000; //change MHz unit
    temp=4400/freq_f;   //for nint calc.
    while(1){
        temp=temp/2;
        i++;
        if(temp<2)break;
    }
    for(j=0;j<i-1;++j){
        ndiv=ndiv*2;
    }
    nint=freq*ndiv/fpfd/1000;
    temp=freq_f*ndiv/fpfd;  //for nfrac calc
    nfrac=nmod*(temp-nint);
    nfrac_i=(uint16_t)round(nfrac);

    //calc gcd by euclid algorithm
    a=nmod;
    b=nfrac_i;
    r=a%b;
    while(r!=0){
        a=b;
        b=r;
        r=a%b;
    }
    
    //calc nfrac and nmodulus
    nfrac_i=nfrac_i/b;
    nmodulus=nmod/b;
    if(nmodulus==1)nmodulus=2;
    
    //calc reg.
    data[0]=0;
    data[1]=0;
    data[4]=0;
    data[0]=(nint<<15)+(nfrac_i<<3)+0;
    data[1]=(1<<27)+(1<<15)+(nmodulus<<3)+1;
    data[4]=(1<<23)+(i<<20)+(200<<12)+(1<<5)+(3<<3)+4;
    
    //spi send
    adc.format(8,0);
    for(i=0;i<5;++i){
        pll_spi_send(ch,data[5-i]);
        thread_sleep_for(3);
    }
    thread_sleep_for(20);
    pll_spi_send(ch,data[0]);
}

void att_set(uint8_t att){
    switch (att) {
        case 0:     //-0dB
            att1=1;
            att2=1;
        break;
        case 1:     //-10dB
            att1=0;
            att2=1;
        break;
        case 2:     //-20dB
            att1=1;
            att2=0;
        break;
        case 3:     //-30dB
            att1=0;
            att2=0;
        break;
    }
}

void port_set(uint8_t port){
    uint8_t stim=port&0b00000011;
    uint8_t det=(port>>2)&0b00000011;
    switch (stim) {
        case 0:     //P1 stim 
            sel=1;
            ref_sw=1;
            sel1=1;
            sel3=1;
        break;
        case 1:     //P2 stim
            sel=1;
            ref_sw=1;
            sel1=0;
            sel3=1;
        break;
        case 2:     //P3 stim
            sel=0;
            ref_sw=0;
            sel1=1;
            sel3=1;
        break;
        case 3:     //P4 stim
            sel=1;
            ref_sw=1;
            sel1=1;
            sel3=0;
        break;
    }
    switch (det) {
        case 0:     //P1 det 
            sig_sw=1;
            sel2=1;
            sel4=1;
        break;
        case 1:     //P2 det
            sig_sw=1;
            sel2=0;
            sel4=1;
        break;
        case 2:     //P3 det
            sig_sw=0;
            sel2=1;
            sel4=1;
        break;
        case 3:     //P4 det
            sig_sw=0;
            sel2=1;
            sel4=0;
        break;
    }
}
