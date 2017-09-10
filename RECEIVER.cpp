#include "RECEIVER.h"
void RECEIVER::begin(int cal){
    printf("Initializing receiver...\n"); 
    int mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC);
    if (mem_fd == -1)printf("Unable to open /dev/mem");
    ring_buffer = (volatile struct ring_buffer*) mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, RCIN_PRUSS_RAM_BASE);
    close(mem_fd);
    ring_buffer->ring_head = 0;
    int calibrated=read_calibration();
    int go=calibrated;
    if(cal==1||calibrated==0)go=0;
    while(go==0){
        go=calibrate();
        if(go==0)printf("Calibration wrong! Can't let you fly with this...\n");
        else printf("Receiver calibrated!\n");
        print_calibration();
    }
    save_calibration();
    printf("Receiver Initialized!\n");
}
int RECEIVER::calibrate(){
    printf("Reseting values and calibrating!\n");
    for(int i=0;i<number_channels;i++){
    mid_points[i]=1000;
    upper_limits[i]=1000;
    lower_limits[i]=1000;
    }
    fd_set readfds;
    struct timeval tv;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
	select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);
    printf("Please release all joysticks, throttle down and press enter when ready...\n");
    while(FD_ISSET(STDIN_FILENO, &readfds)==0){
        printf("Reading... \r");
	    FD_ZERO(&readfds);
	    FD_SET(STDIN_FILENO, &readfds);
	    tv.tv_sec = 0;
	    tv.tv_usec = 20000;
		select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);
		read();
		setMidPoints();
    };
    char msg[10];
    fgets(msg,10,stdin);
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    tv.tv_sec = 1;
    tv.tv_usec = 0;
	select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);
    printf("Mid points calibration done!\n");
    printf("Please move all channels to their maximums/minimums and press enter when ready..\n");
    while(FD_ISSET(STDIN_FILENO, &readfds)==0){
        printf("Reading... \r");
	    FD_ZERO(&readfds);
	    FD_SET(STDIN_FILENO, &readfds);
	    tv.tv_sec = 0;
	    tv.tv_usec = 20000;
		select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);
		read();
		setLimits();
    };
    fgets(msg,10,stdin);
    printf("Maximum/Minimum limits calibrated!\n");
    return check_calibration();
}
void RECEIVER::read(){
    while (ring_buffer->ring_head != ring_buffer->ring_tail){
        if (ring_buffer->ring_tail >= NUM_RING_ENTRIES)return;
    processPulse((ring_buffer->buffer[ring_buffer->ring_head].s1_t) / TICK_PER_US,
    (ring_buffer->buffer[ring_buffer->ring_head].s0_t) / TICK_PER_US);
    ring_buffer->ring_head = (ring_buffer->ring_head + 1) % NUM_RING_ENTRIES;
    }
    offset_channels();
}
void RECEIVER::processPulse(uint16_t width_1,uint16_t width_2){
    uint16_t pulse=width_1+width_2;
    if(pulse>pulse_limit)channel_counter=0;
    else{
        raw_channels[channel_counter]=pulse;
        channel_counter++;
    }
}
void RECEIVER::setMidPoints(){
    for(int i=0;i<number_channels;i++)mid_points[i]=raw_channels[i];
}
void RECEIVER::setLimits(){
		for(int i=0;i<number_channels;i++){
		    if(raw_channels[i]>upper_limits[i])upper_limits[i]=raw_channels[i];
		    if(raw_channels[i]<lower_limits[i])lower_limits[i]=raw_channels[i];
		}
}
void RECEIVER::print_calibration(){
    printMidPoints();
    printLimits();
}
void RECEIVER::printMidPoints(){
    printf("ChX = Mid points\n");
    for(int i=0;i<number_channels;i++){
        printf("Ch%d = %u\n",i,mid_points[i]);
    }
}
void RECEIVER::printLimits(){
    printf("ChX = Max Min\n");
    for(int i=0;i<number_channels;i++){
        printf("Ch%d = %u %u\n",i,upper_limits[i],lower_limits[i]);
    }
}
void RECEIVER::offset_channels(){
    for(int i=0;i<number_channels;i++)channels[i]=raw_channels[i]-mid_points[i];
}
void RECEIVER::save_calibration(){
    FILE *f;
    f=fopen("mid_points_calibration.txt","w");
    for(int i=0;i<number_channels;i++)fprintf(f,"%u\n",mid_points[i]);
    fclose(f);
    f=fopen("upper_limits_calibration.txt","w");
    for(int i=0;i<number_channels;i++)fprintf(f,"%u\n",upper_limits[i]);
    fclose(f);
    f=fopen("lower_limits_calibration.txt","w");
    for(int i=0;i<number_channels;i++)fprintf(f,"%u\n",lower_limits[i]);
    fclose(f);
}
int RECEIVER::read_calibration(){
    int calibrated=1;
    FILE *f;
    if((f=fopen("mid_points_calibration.txt", "r"))==NULL){
        printf("No mid points calibration file found... Calibration required!\n");
        return calibrated=0;
    }
    uint16_t value;
    int i=0;
    fscanf (f, "%hu\n", &value);    
    while (!feof (f))
    {  
        mid_points[i]=value;
        i++;
        fscanf (f, "%hu\n", &value);      
    }
    fclose(f);
    if((f=fopen("upper_limits_calibration.txt", "r"))==NULL){
        printf("No upper limits calibration file found... Calibration required!\n");
        return calibrated=0;
    }
    i=0;
    fscanf (f, "%hu\n", &value);    
    while (!feof (f))
    {  
        upper_limits[i]=value;
        i++;
        fscanf (f, "%hu\n", &value);     
    }
    fclose(f);
    if((f=fopen("lower_limits_calibration.txt", "r"))==NULL){
        printf("No lower limits calibration file found... Calibration required!\n");
        return calibrated=0;
    }
    i=0;
    fscanf (f, "%hu\n", &value);    
    while (!feof (f))
    {  
        lower_limits[i]=value;
        i++;
        fscanf (f, "%hu\n", &value);   
    }
    fclose(f);
    calibrated=check_calibration();
    return calibrated;
}
int RECEIVER::check_calibration(){
    int calibrated=1;
    for(int i=0;i<number_channels;i++){
        if(upper_limits[i]>2000||lower_limits[i]<400||mid_points[i]>upper_limits[i]||mid_points[i]<lower_limits[i])calibrated=0;
    }
    if(calibrated==0){
        printf("Carefull... Needs calibration!\n");
    }
    return calibrated;
}