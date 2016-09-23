/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
 * All information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

// edited by Hyundai Autron
// gcc -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o captureOpenCV.o captureOpenCV.c
// gcc -I. -I./utils `pkg-config opencv --cflags` -I./include  -c -o nvthread.o nvthread.c
// gcc  -o captureOpenCV captureOpenCV.o nvthread.o  -L ./utils -lnvmedia -lnvtestutil_board -lnvtestutil_capture_input -lnvtestutil_i2c -lpthread `pkg-config opencv --libs`

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>

#include <nvcommon.h>
#include <nvmedia.h>

#include <testutil_board.h>
#include <testutil_capture_input.h>

#include "nvthread.h"
#include "car_lib.h"


#include <highgui.h>
#include <cv.h>
#include <ResTable_720To320.h>
#include <pthread.h>
#include <unistd.h>     // for sleep

#define VIP_BUFFER_SIZE 6
#define VIP_FRAME_TIMEOUT_MS 100
#define VIP_NAME "vip"

#define MESSAGE_PRINTF printf

#define CRC32_POLYNOMIAL 0xEDB88320L

#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240

// NEW PARAM
//FOR PARKING
#define DISTANCE_SENSOR
static short frame = 0;
int i = 0;
// NEW PARAM END

static NvMediaVideoSurface *capSurf = NULL;

pthread_cond_t      cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;

int table_298[256];
int table_409[256];
int table_100[256];
int table_208[256];
int table_516[256];

typedef struct
{
    I2cId i2cDevice;
    
    CaptureInputDeviceId vipDeviceInUse;
    NvMediaVideoCaptureInterfaceFormat vipInputtVideoStd;
    unsigned int vipInputWidth;
    unsigned int vipInputHeight;
    float vipAspectRatio;
    
    unsigned int vipMixerWidth;
    unsigned int vipMixerHeight;
    
    NvBool vipDisplayEnabled;
    NvMediaVideoOutputType vipOutputType;
    NvMediaVideoOutputDevice vipOutputDevice[2];
    NvBool vipFileDumpEnabled;
    char * vipOutputFileName;
    
    unsigned int vipCaptureTime;
    unsigned int vipCaptureCount;
} TestArgs;

typedef struct
{
    NvMediaVideoSurface *surf;
    NvBool last;
} QueueElem;

typedef struct
{
    char *name;
    
    NvSemaphore *semStart, *semDone;
    
    NvMediaVideoCapture *capture;
    NvMediaVideoMixer *mixer;
    FILE *fout;
    
    unsigned int inputWidth;
    unsigned int inputHeight;
    
    unsigned int timeout;
    
    NvBool displayEnabled;
    NvBool fileDumpEnabled;
    
    NvBool timeNotCount;
    unsigned int last;
} CaptureContext;

static NvBool stop = NVMEDIA_FALSE;

static void SignalHandler(int signal)
{
    stop = NVMEDIA_TRUE;
    MESSAGE_PRINTF("%d signal received\n", signal);
}

static void GetTime(NvMediaTime *time)
{
    struct timeval t;
    
    gettimeofday(&t, NULL);
    
    time->tv_sec = t.tv_sec;
    time->tv_nsec = t.tv_usec * 1000;
}

static void AddTime(NvMediaTime *time, NvU64 uSec, NvMediaTime *res)
{
    NvU64 t, newTime;
    
    t = (NvU64)time->tv_sec * 1000000000LL + (NvU64)time->tv_nsec;
    newTime = t + uSec * 1000LL;
    res->tv_sec = newTime / 1000000000LL;
    res->tv_nsec = newTime % 1000000000LL;
}

//static NvS64 SubTime(NvMediaTime *time1, NvMediaTime *time2)
static NvBool SubTime(NvMediaTime *time1, NvMediaTime *time2)
{
    NvS64 t1, t2, delta;
    
    t1 = (NvS64)time1->tv_sec * 1000000000LL + (NvS64)time1->tv_nsec;
    t2 = (NvS64)time2->tv_sec * 1000000000LL + (NvS64)time2->tv_nsec;
    delta = t1 - t2;
    
    //    return delta / 1000LL;
    return delta > 0LL;
}


static void DisplayUsage(void)
{
    printf("Usage : nvmedia_capture [options]\n");
    printf("Brief: Displays this help if no arguments are given. Engages the respective capture module whenever a single \'c\' or \'v\' argument is supplied using default values for the missing parameters.\n");
    printf("Options:\n");
    printf("-va <aspect ratio>    VIP aspect ratio (default = 1.78 (16:9))\n");
    printf("-vmr <width>x<height> VIP mixer resolution (default 800x480)\n");
    printf("-vf <file name>       VIP output file name; default = off\n");
    printf("-vt [seconds]         VIP capture duration (default = 10 secs); overridden by -vn; default = off\n");
    printf("-vn [frames]          # VIP frames to be captured (default = 300); default = on if -vt is not used\n");
}

static int ParseOptions(int argc, char *argv[], TestArgs *args)
{
    int i = 1;
    
    // Set defaults if necessary - TBD
    args->i2cDevice = I2C4;     // i2c chnnel
    
    args->vipDeviceInUse = AnalogDevices_ADV7182;
    args->vipInputtVideoStd = NVMEDIA_VIDEO_CAPTURE_INTERFACE_FORMAT_VIP_NTSC;
    args->vipInputWidth = 720;
    args->vipInputHeight = 480;
    args->vipAspectRatio = 0.0f;
    
    args->vipMixerWidth = 800;
    args->vipMixerHeight = 480;
    
    args->vipDisplayEnabled = NVMEDIA_FALSE;
    args->vipOutputType = NvMediaVideoOutputType_OverlayYUV;
    args->vipOutputDevice[0] = NvMediaVideoOutputDevice_LVDS;
    args->vipFileDumpEnabled = NVMEDIA_FALSE;
    args->vipOutputFileName = NULL;
    
    args->vipCaptureTime = 0;
    args->vipCaptureCount = 0;
    
    
    
    if(i < argc && argv[i][0] == '-')
    {
        while(i < argc && argv[i][0] == '-')
        {
            if(i > 1 && argv[i][1] == '-')
            {
                MESSAGE_PRINTF("Using basic and custom options together is not supported\n");
                return 0;
            }
            
            // Get options
            if(!strcmp(argv[i], "-va"))
            {
                if(++i < argc)
                {
                    if(sscanf(argv[i], "%f", &args->vipAspectRatio) != 1 || args->vipAspectRatio <= 0.0f) // TBC
                    {
                        MESSAGE_PRINTF("Bad VIP aspect ratio: %s\n", argv[i]);
                        return 0;
                    }
                }
                else
                {
                    MESSAGE_PRINTF("Missing VIP aspect ratio\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vmr"))
            {
                if(++i < argc)
                {
                    if(sscanf(argv[i], "%ux%u", &args->vipMixerWidth, &args->vipMixerHeight) != 2)
                    {
                        MESSAGE_PRINTF("Bad VIP mixer resolution: %s\n", argv[i]);
                        return 0;
                    }
                }
                else
                {
                    MESSAGE_PRINTF("Missing VIP mixer resolution\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vf"))
            {
                args->vipFileDumpEnabled = NVMEDIA_TRUE;
                if(++i < argc)
                    args->vipOutputFileName = argv[i];
                else
                {
                    MESSAGE_PRINTF("Missing VIP output file name\n");
                    return 0;
                }
            }
            else if(!strcmp(argv[i], "-vt"))
            {
                if(++i < argc)
                    if(sscanf(argv[i], "%u", &args->vipCaptureTime) != 1)
                    {
                        MESSAGE_PRINTF("Bad VIP capture duration: %s\n", argv[i]);
                        return 0;
                    }
            }
            else if(!strcmp(argv[i], "-vn"))
            {
                if(++i < argc)
                    if(sscanf(argv[i], "%u", &args->vipCaptureCount) != 1)
                    {
                        MESSAGE_PRINTF("Bad VIP capture count: %s\n", argv[i]);
                        return 0;
                    }
            }
            else
            {
                MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
                return 0;
            }
            
            i++;
        }
    }
    
    if(i < argc)
    {
        MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
        return 0;
    }
    
    // Check for consistency
    if(i < 2)
    {
        DisplayUsage();
        return 0;
    }
    
    
    if(args->vipAspectRatio == 0.0f)
        args->vipAspectRatio = 1.78f;
    
    if(!args->vipDisplayEnabled && !args->vipFileDumpEnabled)
        args->vipDisplayEnabled = NVMEDIA_TRUE;
    
    
    if(!args->vipCaptureTime && !args->vipCaptureCount)
        args->vipCaptureCount = 300;
    else if(args->vipCaptureTime && args->vipCaptureCount)
        args->vipCaptureTime = 0;
    
    
    
    return 1;
}

static int DumpFrame(FILE *fout, NvMediaVideoSurface *surf)
{
    NvMediaVideoSurfaceMap surfMap;
    unsigned int width, height;
    
    if(NvMediaVideoSurfaceLock(surf, &surfMap) != NVMEDIA_STATUS_OK)
    {
        MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in DumpFrame()\n");
        return 0;
    }
    
    width = surf->width;
    height = surf->height;
    
    unsigned char *pY[2] = {surfMap.pY, surfMap.pY2};
    unsigned char *pU[2] = {surfMap.pU, surfMap.pU2};
    unsigned char *pV[2] = {surfMap.pV, surfMap.pV2};
    unsigned int pitchY[2] = {surfMap.pitchY, surfMap.pitchY2};
    unsigned int pitchU[2] = {surfMap.pitchU, surfMap.pitchU2};
    unsigned int pitchV[2] = {surfMap.pitchV, surfMap.pitchV2};
    unsigned int i, j;
    
    for(i = 0; i < 2; i++)
    {
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pY[i], width, 1, fout);
            pY[i] += pitchY[i];
        }
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pU[i], width / 2, 1, fout);
            pU[i] += pitchU[i];
        }
        for(j = 0; j < height / 2; j++)
        {
            fwrite(pV[i], width / 2, 1, fout);
            pV[i] += pitchV[i];
        }
    }
    
    
    NvMediaVideoSurfaceUnlock(surf);
    
    return 1;
}

static int Frame2Ipl(IplImage* img)
{
    NvMediaVideoSurfaceMap surfMap;
    unsigned int resWidth, resHeight;
    int r,g,b;
    unsigned char y,u,v;
    int num;
    
    if(NvMediaVideoSurfaceLock(capSurf, &surfMap) != NVMEDIA_STATUS_OK)
    {
        MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in Frame2Ipl()\n");
        return 0;
    }
    
    unsigned char *pY[2] = {surfMap.pY, surfMap.pY2};
    unsigned char *pU[2] = {surfMap.pU, surfMap.pU2};
    unsigned char *pV[2] = {surfMap.pV, surfMap.pV2};
    unsigned int pitchY[2] = {surfMap.pitchY, surfMap.pitchY2};
    unsigned int pitchU[2] = {surfMap.pitchU, surfMap.pitchU2};
    unsigned int pitchV[2] = {surfMap.pitchV, surfMap.pitchV2};
    unsigned int i, j, k, x;
    unsigned int stepY, stepU, stepV;
    
    resWidth = RESIZE_WIDTH;
    resHeight = RESIZE_HEIGHT;
    
    // Frame2Ipl
    img->nSize = 112;
    img->ID = 0;
    img->nChannels = 3;
    img->alphaChannel = 0;
    img->depth = IPL_DEPTH_8U;    // 8
    img->colorModel[0] = 'R';
    img->colorModel[1] = 'G';
    img->colorModel[2] = 'B';
    img->channelSeq[0] = 'B';
    img->channelSeq[1] = 'G';
    img->channelSeq[2] = 'R';
    img->dataOrder = 0;
    img->origin = 0;
    img->align = 4;
    img->width = resWidth;
    img->height = resHeight;
    img->imageSize = resHeight*resWidth*3;
    img->widthStep = resWidth*3;
    img->BorderMode[0] = 0;
    img->BorderMode[1] = 0;
    img->BorderMode[2] = 0;
    img->BorderMode[3] = 0;
    img->BorderConst[0] = 0;
    img->BorderConst[1] = 0;
    img->BorderConst[2] = 0;
    img->BorderConst[3] = 0;
    
    stepY = 0;
    stepU = 0;
    stepV = 0;
    i = 0;
    
    for(j = 0; j < resHeight; j++)
    {
        for(k = 0; k < resWidth; k++)
        {
            x = ResTableX_720To320[k];
            y = pY[i][stepY+x];
            u = pU[i][stepU+x/2];
            v = pV[i][stepV+x/2];
            
            // YUV to RGB
            r = y + 1.4075*(v-128);
            g = y - 0.34455*(u-128) - 0.7169*(v-128);
            b = y + 1.779*(u-128);
            
            
            r = r>255? 255 : r<0 ? 0 : r;
            g = g>255? 255 : g<0 ? 0 : g;
            b = b>255? 255 : b<0 ? 0 : b;
            
            
            num = 3*k+3*resWidth*(j);
            img->imageData[num] = b;
            img->imageData[num+1] = g;
            img->imageData[num+2] = r;
            //img->imageDataOrigin[num] = b;
            //img->imageDataOrigin[num+1] = g;
            //img->imageDataOrigin[num+2] = r;
            
            
        }
        stepY += pitchY[i];
        stepU += pitchU[i];
        stepV += pitchV[i];
    }
    
    
    NvMediaVideoSurfaceUnlock(capSurf);
    
    return 1;
}

static unsigned int CaptureThread(void *params)
{
    int i = 0;
    NvU64 stime, ctime;
    NvMediaTime t1 = {0}, t2 = {0}, st = {0}, ct = {0};
    CaptureContext *ctx = (CaptureContext *)params;
    NvMediaVideoSurface *releaseList[4] = {NULL}, **relList;
    NvMediaRect primarySrcRect;
    NvMediaPrimaryVideo primaryVideo;
    
    primarySrcRect.x0 = 0;
    primarySrcRect.y0 = 0;
    primarySrcRect.x1 = ctx->inputWidth;
    primarySrcRect.y1 = ctx->inputHeight;
    
    primaryVideo.next = NULL;
    primaryVideo.previous = NULL;
    primaryVideo.previous2 = NULL;
    primaryVideo.srcRect = &primarySrcRect;
    primaryVideo.dstRect = NULL;
    
    
    NvSemaphoreDecrement(ctx->semStart, NV_TIMEOUT_INFINITE);
    
    if(ctx->timeNotCount)
    {
        GetTime(&t1);
        AddTime(&t1, ctx->last * 1000000LL, &t1);
        GetTime(&t2);
        printf("timeNotCount\n");
    }
    GetTime(&st);
    stime = (NvU64)st.tv_sec * 1000000000LL + (NvU64)st.tv_nsec;
    
    while((ctx->timeNotCount? (SubTime(&t1, &t2)): ((unsigned int)i < ctx->last)) && !stop)
    {
        GetTime(&ct);
        ctime = (NvU64)ct.tv_sec * 1000000000LL + (NvU64)ct.tv_nsec;
        //printf("frame=%3d, time=%llu.%09llu[s] \n", i, (ctime-stime)/1000000000LL, (ctime-stime)%1000000000LL);
        
        pthread_mutex_lock(&mutex);            // for ControlThread()
        
        if(!(capSurf = NvMediaVideoCaptureGetFrame(ctx->capture, ctx->timeout)))
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoCaptureGetFrame() failed in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
            break;
        }
        
        if(i%3 == 0)                        // once in three loop = 10 Hz
            pthread_cond_signal(&cond);        // ControlThread() is called
        
        pthread_mutex_unlock(&mutex);        // for ControlThread()
        
        primaryVideo.current = capSurf;
        primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_TOP_FIELD;
        
        if(NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
                                                        NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                                                        NULL, // background
                                                        &primaryVideo, // primaryVideo
                                                        NULL, // secondaryVideo
                                                        NULL, // graphics0
                                                        NULL, // graphics1
                                                        releaseList, // releaseList
                                                        NULL)) // timeStamp
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the top field in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
        }
        
        primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_BOTTOM_FIELD;
        if(NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
                                                        NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                                                        NULL, // background
                                                        &primaryVideo, // primaryVideo
                                                        NULL, // secondaryVideo
                                                        NULL, // graphics0
                                                        NULL, // graphics1
                                                        releaseList, // releaseList
                                                        NULL)) // timeStamp
        { // TBD
            MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the bottom field in %sThread\n", ctx->name);
            stop = NVMEDIA_TRUE;
        }
        
        if(ctx->fileDumpEnabled)
        {
            if(!DumpFrame(ctx->fout, capSurf))
            { // TBD
                MESSAGE_PRINTF("DumpFrame() failed in %sThread\n", ctx->name);
                stop = NVMEDIA_TRUE;
            }
            
            if(!ctx->displayEnabled)
                releaseList[0] = capSurf;
        }
        
        relList = &releaseList[0];
        
        while(*relList)
        {
            if(NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
            { // TBD
                MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);
                stop = NVMEDIA_TRUE;
                break;
            }
            relList++;
        }
        
        if(ctx->timeNotCount)
            GetTime(&t2);
        
        i++;
    } // while end
    
    // Release any left-over frames
    //    if(ctx->displayEnabled && capSurf && capSurf->type != NvMediaSurfaceType_YV16x2) // To allow returning frames after breaking out of the while loop in case of error
    if(ctx->displayEnabled && capSurf)
    {
        NvMediaVideoMixerRender(ctx->mixer, // mixer
                                NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
                                NULL, // background
                                NULL, // primaryVideo
                                NULL, // secondaryVideo
                                NULL, // graphics0
                                NULL, // graphics1
                                releaseList, // releaseList
                                NULL); // timeStamp
        
        relList = &releaseList[0];
        
        while(*relList)
        {
            if(NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
                MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);
            
            relList++;
        }
    }
    
    NvSemaphoreIncrement(ctx->semDone);
    return 0;
}

static void CheckDisplayDevice(NvMediaVideoOutputDevice deviceType, NvMediaBool *enabled, unsigned int *displayId)
{
    int outputDevices;
    NvMediaVideoOutputDeviceParams *outputParams;
    int i;
    
    // By default set it as not enabled (initialized)
    *enabled = NVMEDIA_FALSE;
    *displayId = 0;
    
    // Get the number of devices
    if(NvMediaVideoOutputDevicesQuery(&outputDevices, NULL) != NVMEDIA_STATUS_OK) {
        return;
    }
    
    // Allocate memory for information for all devices
    outputParams = malloc(outputDevices * sizeof(NvMediaVideoOutputDeviceParams));
    if(!outputParams) {
        return;
    }
    
    // Get device information for acll devices
    if(NvMediaVideoOutputDevicesQuery(&outputDevices, outputParams) != NVMEDIA_STATUS_OK) {
        free(outputParams);
        return;
    }
    
    // Find desired device
    for(i = 0; i < outputDevices; i++) {
        if((outputParams + i)->outputDevice == deviceType) {
            // Return information
            *enabled = (outputParams + i)->enabled;
            *displayId = (outputParams + i)->displayId;
            break;
        }
    }
    
    // Free information memory
    free(outputParams);
}


////////// Team.c --> BBC
IplImage** binary_image_array(IplImage *img, int *steer_y, int *hill_y, int *bump_y, int *bump2, int *final, int * cnt_final, int * dashline_on, int *stopline_on)
{
    IplImage** results = (IplImage**) malloc( sizeof(IplImage*) * 3);
    int h = img->height;
    int w = img->width;
    int d = img->nChannels;
    int i,j,k, ii, jj;
    unsigned char R,G,B;
    *cnt_final = 0;
    
    unsigned char RR,GG,BB;
    int aavg = 0;
    
    int gp1, gp2, gp3, gp4;
    int gp2_up;
    int avg=0;
    
    results[0] = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
    results[1] = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
    results[2] = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,3);
    
    int hill_stop = 0;
    int hill_stop2 = 0;
    int bump_stop = 0;
    int final_stop = 0;
    int turnback = 0;
    int pos[3] = {0,};
    int a = 0;
    int posSum =0;

    
    for( i = 0 ; i < w; i++ ){
        hill_stop = 0;
        hill_stop2 = 0;
        bump_stop = 0;
        final_stop = 0;
        
        
        //if dashline is on or your rotary-> find black
        if(*dashline_on == 1 || *stopline_on == 2){
            
            if(i < 110) a = 0;
            else if (i < 210) a = 1;
            else a = 2;
            
            for(j = h-1; j >= 20; j--){ //can be half
                B = (uchar)img->imageData[d*w*j + d*i + 0];
                G = (uchar)img->imageData[d*w*j + d*i + 1];
                R = (uchar)img->imageData[d*w*j + d*i + 2];
                
                avg = (R + G + B)/3;
                
                if(avg < 50){
                    gp2 = 255;
                    pos[a] ++;
                }
                else
                    gp2 = 0;
                
                
                results[1]->imageData[d*w*j + d*i + 0] = (uchar)gp2;
                results[1]->imageData[d*w*j + d*i + 1] = (uchar)gp2;
                results[1]->imageData[d*w*j + d*i + 2] = (uchar)gp2;
            }
        }
        
        
        for( j = h-1; j>=0; j-- ){
            B = (uchar)img->imageData[d*w*j + d*i + 0];
            G = (uchar)img->imageData[d*w*j + d*i + 1];
            R = (uchar)img->imageData[d*w*j + d*i + 2];
            
            gp1 = (500*R + 500*G - 800*B)/1000;
            gp2 = gp1;
            gp3 = gp1;
            avg = (R + G + B)/3;
            
            if (gp1<60)
                gp1=0;
            else
                gp1=255;
            
            gp4 = gp1;
          
	//you can delete
	if((*stopline_on >= 3 && *stopline_on <= 5) || (*dashline_on <= -10)){
          if (avg < 200)
                gp1=gp1 > 0 ? gp1 : 0;
            else
                gp1 =255;
        }
            results[0]->imageData[d*w*j + d*i + 0] = (uchar)gp1;
            results[0]->imageData[d*w*j + d*i + 1] = (uchar)gp1;
            results[0]->imageData[d*w*j + d*i + 2] = (uchar)gp1;
            
	if(avg > 200)
		gp3 = 255;
	else
		gp3 = 0;
	if(B < 200)
                gp3 = 0;

            /*if(gp3 < 60)
                gp3 = 0;
            else
                gp3 = 255;
            
            if(avg<200)
                gp3 = gp3 > 0 ? gp3 : 0;
            else
                gp3 = 255;
            
            if(B < 200)
                gp3 = 0;
            */
            results[2]->imageData[d*w*j + d*i + 0] = (uchar)gp3;
            results[2]->imageData[d*w*j + d*i + 1] = (uchar)gp3;
            results[2]->imageData[d*w*j + d*i + 2] = (uchar)gp3;
            
            // ######## steer_y & hill_y ###########
            if(gp1 > 170 && hill_stop == 0){
                //hill_y[i] = j;
                hill_stop = 1;
                if(j >= (h/2))
                    steer_y[i] = j - (h/2);
            }
            
            if( gp4 > 170 && hill_stop2 == 0){
                hill_y[i] = j;
                hill_stop2 = 1;
            }
            
            
            if(gp2 > 170 && bump_stop == 0 && j >= h/3 + 30){
                bump_stop = 1;
                bump_y[i] = j - h/3;
                
                BB = (uchar)img->imageData[d*w*(j-30) + d*i + 0];
                GG = (uchar)img->imageData[d*w*(j-30) + d*i + 1];
                RR = (uchar)img->imageData[d*w*(j-30) + d*i + 2];
                
                gp2_up = (500*RR + 500*GG - 800*BB)/1000;
                aavg = (RR + GG + BB)/3;
                
                if(gp2_up < 60)
                    gp2_up = 0;
                else
                    gp2_up = 255;
                
                if(aavg < 150)
                    gp2_up = gp2_up > 0 ? gp2_up : 0;
                else
                    gp2_up = 255;
                
                bump2[i] = gp2_up;
            }
            
            if(gp2 == 0 && j ==160 ){
                (*cnt_final) ++;
                
                
            }
            // ###### final line #######
            if( j >= (h/2) && final_stop == 0 && gp3 >= 200){
                final_stop = 1;
                final[i] = j - (h/2);
            }
        }
        
    }
    
	/*
	10/16 EDIT
	회전교차로에서 정지선앞에 멈췄을 때
	화면을 가로로 3등분해서  pos[0] | pos[1] | pos[2] 로 그림자(검은색)을 구한 배열 만들어놓고...
	posSum 은 오른쪽 2/3를 합한 값
	정지선에 멈췄을 때 1/3지점까지는 물체가 없고, 오른쪽 2/3지점에 물체가 감지되면 출발한다.
	회전교차로 안에서 전방장애물을 감지해서 속도조절하는 것은 미구현
	
	*/
    //STOPLINE FIND CAR
    if(*stopline_on == 2){
	posSum = pos[1] + pos[2];

	if( pos[0] < 300 && posSum > 1000 && posSum < 2500){
		moveSpeed(1515, 120);
		*stopline_on = 3;
	}
    }	

    //DASHLINE find minimum -> dd
    if(*dashline_on == 1){
        int min = pos[0] < pos[1] ? ( pos[0] < pos[2] ? pos[0] : pos[2] ) : ( pos[1] < pos[2] ? pos[1] : pos[2]) ;
        int m = 0;
        for( m=0 ; m<3; m++)
            if( min == pos[m] )
                *dashline_on = 10 + m;
        
        printf("$$$$*$$&$*$$*$&*$*$&$*&$&$*$*%d %d %d %d I find the way!~~\n",pos[0], pos[1], pos[2], *dashline_on);
        
    }
    
    if(*dashline_on <= -10){
        
        //gp4 yellow alot -> turn back
        for( ii=0 ; ii< h; ii++){
            turnback =0;
            for(jj =0 ; jj<w ; jj++){
                B = (uchar)img->imageData[d*w*ii + d*jj + 0];
                G = (uchar)img->imageData[d*w*ii + d*jj + 1];
                R = (uchar)img->imageData[d*w*ii + d*jj + 2];
                
                gp1 = (500*R + 500*G - 800*B)/1000;
                
                avg = (R + G + B)/3;
                
                if (gp1<60)
                    gp1=0;
                else
                    gp1=255;
                
                
                if (avg<220)
                    gp1=gp1 > 0 ? gp1 : 0;
                else
                    gp1=255;
                
                if (gp1 == 255)
                    turnback ++;
                
            }
            
            if( turnback > 170){
                printf(" TurnBack : %d   , h : %d\n", turnback , ii );
                moveSpeed(1510 - 400 * (*dashline_on + 11), 150);
                Winker_Write(ALL_ON);
                usleep(1000000);
                Winker_Write(ALL_OFF);
		
                *dashline_on = -1;
                break;
            }
            
        }
    }
    
    
    
    return results;
}

void init_array(int *input, int len, int value)
{
    int i;
    for(i = 0; i < len; i++){
        input[i] = value;
    }
}

int isRed(IplImage *img, int x, int y){
    unsigned char r = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+2];
    unsigned char g = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+1];
    unsigned char b = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+0];
    if(r > 100 && g < 60 && b < 75) { return 1;}
    return 0;
}

int red_detect(IplImage *img, int *red_stop){
    
    int check = 0,check2 = 0;
    int increase = 0;
    int decrease = 0;
    int cnt = 0,cnt2=0;
    int i,j;
    int red_x[100];
    int red_right_x[100];
    
    red_x[0]=0;
    red_right_x[0]=0;
    
    int step = 8;
    
    for(j=0;j<img->height;j+=step){
        for(i=0;i<img->width;i++){
            if(isRed(img,i,j)==1){
                //cvCircle(img,cvPoint(i,j),3,CV_RGB(255,0,0),-1,8,0);
                red_x[cnt++] = i;
                break;
            }
        }
        
        for(i=img->width-1;i>=0;i--){
            if(isRed(img,i,j)==1){
                //cvCircle(img,cvPoint(i,j),3,CV_RGB(0,255,0),-1,8,0);
                red_right_x[cnt2++] = i;
                break;
            }
        }
    }
    
    //printf("cnt : [%d] , cnt2 : [%d], RED_STOP[%d]\n", cnt, cnt2, *red_stop);
    
    if(cnt>=5 && cnt2 >= 5){
        
        for( i = 1 ; i < cnt ; i++ ){
            if(abs(red_x[i]-red_x[i-1]) < 4 ){
                increase++;
            }
        }
        
        if(cnt == 4 && increase >= cnt -1){
            check = 1;
        }else if(cnt2 >=5 && cnt <= 6 && increase >= cnt -2){
            check = 1;
        } else if(cnt >6 && increase >= cnt - 3){
            check = 1;
        }
        
        for( i = 1 ; i < cnt2 ; i++ ){
            if(abs(red_right_x[i]-red_right_x[i-1]) < 4 ){
                decrease++;
            }
        }
        
        printf("inc : [%d], dec : [%d]\n", increase, decrease);
        
        if(cnt2 == 4 && decrease >= cnt2 -1){
            check2 = 1;
        }else if(cnt2 >=5 && cnt2 <= 6 && decrease >= cnt2 -2){
            check2 = 1;
        }else if(cnt >6 && decrease >= cnt2 - 3){
            check2 = 1;
        }
        
        if( cnt >= 16 && cnt2 >= 16 && decrease >= 13){
            *red_stop = 1;
            return 1;
        }
        if((check * check2) == 1)
            *red_stop = 1;
        return (check * check2);
        
    }
    else if( cnt <= 0 && cnt2 <= 0 && *red_stop == 1){
        char fileName[100];
        sprintf(fileName, "red/imgOrigin%d.png", i);
        cvSaveImage(fileName, img, 0);
        
        printf("shit!!!!\n");
        *red_stop = 0;
        return 0;
    }
    return 0;
}

void checkline(int * y,int num_y, int * left_startend, int * right_startend,float * left_tangent,float * right_tangent)
{
    
    int ii,jj;
    int findstartend_left = 0;
    int findstartend_right = 0;
    left_startend[0] = 319;
    // finding start and end of both lines
    for(ii = 0 ; ii < num_y; ii++){
        if ( y[ii] != 1000 && findstartend_left == 0 )
        {
            left_startend[0] = ii;
            findstartend_left = 1;
            
        }
        else if ( y[ii] == 1000  &&findstartend_left ==1)
        {
            if(abs(y[ii+9]-y[ii-1])<10){
                ii +=10;
                continue;
            }
            left_startend[1] = ii-1;
            break;
        }
        else if( abs(y[ii+1]-y[ii])>10 && findstartend_left ==1)
        {
            left_startend[1] = ii;
            break;
        }
        else if ( ii == num_y -1)
            left_startend[1] = ii;
        
    }
    
    for(ii = num_y -1 ; ii >= 0; ii --){
        if (  y[ii]!= 1000 && findstartend_right == 0 )
        {
            right_startend[1] = ii;
            findstartend_right = 1;
            
        }
        else if ( y[ii] == 1000 && findstartend_right ==1)
        {
            if(abs(y[ii+1]-y[ii-9])<10){
                ii -=10;
                continue;
            }
            right_startend[0] = ii+1;
            break;
        }
        else if( abs(y[ii-1]-y[ii])>10 && findstartend_right ==1)
        {
            right_startend[0] = ii;
            break;
        }
        else if ( ii == 0)
            right_startend[0] = ii;
        
    }
    
    int sum_xl = 0;
    int sum_yl = 0;
    int sum_xyl = 0;
    int sum_xxl = 0;
    int sum_xr = 0;
    int sum_yr = 0;
    int sum_xyr = 0;
    int sum_xxr = 0;
    int tt,rr;
    // finding tangent y = ax + b => a = (N* sum(x*y)-sum(x)*sum(y))/(N*sum(x^2)-sum(x)*sum(x))
    if(left_startend[1]-left_startend[0] < 5)
        * left_tangent = 1000;
    else
    {
        for(ii = left_startend[0] ; ii <= left_startend[1] ; ii ++)
        {
            sum_xl += ii;
            sum_yl += (120 - y[ii]);
            sum_xyl += ii * (120 - y[ii]);
            sum_xxl += ii*ii;
            
        }
        
        tt  = (- sum_xl * sum_yl + (left_startend[1]-left_startend[0]+1) * sum_xyl);
        rr  = ((left_startend[1]-left_startend[0]+1)*sum_xxl - sum_xl*sum_xl);
        * left_tangent = (float)tt/(float)rr;
    }
    if(right_startend[1]-right_startend[0] < 5)
        * right_tangent = 1000;
    else
    {
        for(ii = right_startend[0] ; ii <= right_startend[1] ; ii ++)
        {
            sum_xr += ii;
            sum_yr += (120 - y[ii]);
            sum_xyr += ii * (120 - y[ii]);
            sum_xxr += ii*ii;
            
        }
        
        tt  = (- sum_xr * sum_yr + (right_startend[1]-right_startend[0]+1) * sum_xyr);
        rr  = ((right_startend[1]-right_startend[0]+1)*sum_xxr - sum_xr*sum_xr);
        * right_tangent = (float)tt/(float)rr;
    }
    
}
int checkbump(int *bump2, int num_y, int *tomsteer, int midsteer, int bump_on)
{
    if(bump_on ==0|| bump_on ==2)
        return 0;
    int ii,jj;
    int startend[2] = {0,0};
    int onoff = 0;
    int step = 30;
    
    
    for(ii = 0 ; ii < num_y ; ii++)
    {
        if(onoff ==1 && ii == 319)
            startend[1] = ii;
        
        if (onoff ==0 && bump2[ii] == 255){
            onoff =1;
            startend[0] = ii;
            continue;
        }
        
        if (onoff == 1 && bump2[ii] == 0){
            if( ii< 310 &&  bump2[ii+1] == 255){
                ii += 1;
                printf("plus 1");
                continue;}
            else if(ii< 310 && bump2[ii+2]== 255){
                ii += 2;
                printf("plus 2");
                continue;}
            else if(ii< 310 && bump2[ii+3] == 255){
                ii += 3;
                printf("plus 3");
                continue;}
            else if(ii< 310 && bump2[ii+4]== 255){
                ii += 4;
                printf("plus 4");
                continue;}
            else if(ii< 310 && bump2[ii+5] == 255){
                ii += 5;
                printf("plus 5");
                continue;}
            else if(ii< 310 && bump2[ii+6] == 255){
                ii += 6;
                printf("plus 6");
                continue;}
            else if(ii< 310 && bump2[ii+7] == 255){
                ii += 7;
                printf("plus 7");
                continue;}
            else if(ii< 310 && bump2[ii+8]== 255){
                ii += 8;
                printf("plus 8");
                continue;}
            else if(ii< 310 && bump2[ii+9] == 255){
                ii += 9;
                printf("plus 9");
                continue;}
            else if(ii< 310 && bump2[ii+10] == 255){
                ii += 10;
                printf("plus 10");
                continue;}
            else {
                startend[1] = ii - 1;
                break;
            }
        }
    }
    
    //printf("[i] startend : (%d,%d) \n",i, startend[0],startend[1]);
    
    if (startend[1]-startend[0] < 160)
        return 0;
    else{
        printf("\n*****************bump*******************startend : (%d,%d) \n",startend[0],startend[1]);
        
        /*OLD SERVO STEERING
         
         * tomsteer = midsteer + 3 * ( 160 - (startend[0] + startend[1])/2) ;
         * tomsteer = *tomsteer > 2000 ? 2000 : ( *tomsteer < 1000 ? 1000 : *tomsteer);
         
         */
        
        //NEW SERVO STEERING
        
        * tomsteer = midsteer + 3 * ( 160 - (startend[0] + startend[1])/2) ;
        * tomsteer = *tomsteer > 1900 ? 1900 : ( *tomsteer < 1100 ? 1100 : *tomsteer);
        
    }
    return 1;
}


void checkhill(int * y, int num_y,int * hillcnt,int * hillrestraint,int * is_vpark_on){
    
    int left_startend[2];
    int right_startend[2];
    float left_tangent;
    float right_tangent;
    
    int ii,jj;
    int findstartend_left = 0;
    int findstartend_right = 0;
    left_startend[0] = 319;
    // finding start and end of both lines
    for(ii = 0 ; ii < num_y; ii++){
        if ( y[ii] != 1000 && findstartend_left == 0 )
        {
            left_startend[0] = ii;
            findstartend_left = 1;
            
        }
        else if ( y[ii] == 1000  &&findstartend_left ==1)
        {
            if(abs(y[ii+9]-y[ii-1])<10){
                ii +=10;
                continue;
            }
            left_startend[1] = ii-1;
            break;
        }
        else if( abs(y[ii+1]-y[ii])>10 && findstartend_left ==1)
        {
            left_startend[1] = ii;
            break;
        }
        else if ( ii == num_y -1)
            left_startend[1] = ii;
        
    }
    /*
     for(ii = num_y -1 ; ii >= 0; ii --){
     if (  y[ii]!= 1000 && findstartend_right == 0 )
     {
     right_startend[1] = ii;
     findstartend_right = 1;
     
     }
     else if ( y[ii] == 1000 && findstartend_right ==1)
     {
     if(abs(y[ii+1]-y[ii-9])<10){
					ii -=10;
					continue;
     }
     right_startend[0] = ii+1;
     break;
     }
     else if( abs(y[ii-1]-y[ii])>10 && findstartend_right ==1)
     {
     right_startend[0] = ii;
     break;
     }
     else if ( ii == 0)
     right_startend[0] = ii;
     
     }
     */
    int sum_xl = 0;
    int sum_yl = 0;
    int sum_xyl = 0;
    int sum_xxl = 0;
    int sum_xr = 0;
    int sum_yr = 0;
    int sum_xyr = 0;
    int sum_xxr = 0;
    int tt,rr;
    // finding tangent y = ax + b => a = (N* sum(x*y)-sum(x)*sum(y))/(N*sum(x^2)-sum(x)*sum(x))
    if(left_startend[1]-left_startend[0] < 20)
        left_tangent = 1000;
    else
    {
        for(ii = left_startend[0] ; ii <= left_startend[1] ; ii ++)
        {
            sum_xl += ii;
            sum_yl += (240 - y[ii]);
            sum_xyl += ii * (240 - y[ii]);
            sum_xxl += ii*ii;
            
        }
        
        tt  = (- sum_xl * sum_yl + (left_startend[1]-left_startend[0]+1) * sum_xyl);
        rr  = ((left_startend[1]-left_startend[0]+1)*sum_xxl - sum_xl*sum_xl);
        left_tangent = (float)tt/(float)rr;
    }
    /*
     if(right_startend[1]-right_startend[0] < 3)
     right_tangent = 1000;
     else
     {
     for(ii = right_startend[0] ; ii <= right_startend[1] ; ii ++)
     {
     sum_xr += ii;
     sum_yr += (240 - y[ii]);
     sum_xyr += ii * (240 - y[ii]);
     sum_xxr += ii*ii;
     
     }
     
     tt  = (- sum_xr * sum_yr + (right_startend[1]-right_startend[0]+1) * sum_xyr);
     rr  = ((right_startend[1]-right_startend[0]+1)*sum_xxr - sum_xr*sum_xr);
     right_tangent = (float)tt/(float)rr;
     }
     */
    printf("hill : (%f)",left_tangent);
    
    if(left_tangent<2 && left_tangent > 0.7 && *hillcnt ==0 ){
        *hillcnt = 1;
    }
    else if(left_tangent<0.8 && left_tangent > 0.7 && *hillcnt ==1){
        *hillcnt = 1;
    }
    else if(left_tangent<2 && left_tangent > 0.8 && *hillcnt ==1){
        *hillcnt =2;
    }
    else if(left_tangent<0.85 && left_tangent > 0.8 && *hillcnt ==2){
        *hillcnt = 2;
    }
    else if(left_tangent<2 && left_tangent > 0.87 && *hillcnt ==2 &&left_startend[1]-left_startend[0] >80){
        *hillcnt = 3;
    }
    else if(*hillcnt ==3 && *hillrestraint>0){
        *hillrestraint = *hillrestraint -1;
        if(*hillrestraint ==1){
            DesireSpeed_Write(150);
            *is_vpark_on =2;
            *hillcnt = 0;
        }
    }
    else{
        *hillcnt = 0;
    }
    
}

int isLine(IplImage *img, int x, int y){
    unsigned char r = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+2];
    unsigned char g = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+1];
    unsigned char b = (uchar)img->imageData[img->widthStep*y+img->nChannels*x+0];
    if(r == 255 && g == 255 && b == 255) { return 1;}
    return 0;
}

void checkdir(int * y, int * left_startend,int *right_startend,float * l,float * r,int *tomsteer, int midsteer, IplImage *bimgs)
{
    int steer ;//= 1510;
    int countnull = 0, ii;
    int straightsteer_threshold = 80;
    int turnsteer_threshold = 300;
    int turn_threshold = 55;
    
    int diff0 = right_startend[0] -left_startend[0];
    int diff1 = right_startend[1] - left_startend[1];
    
    
    for(ii = 0 ; ii < 320; ii++){
        if (y[ii] == 1000)
            countnull ++;
    }
    
    //y = ax + b ;; right,left
    //coordinate is ordinary cartesian coordinate : (0,0) => left bottom
    float al,bl,ar,br;
    al = *l ;
    bl = - al * left_startend[0] + (120 - y[left_startend[0]]);
    ar = *r ;
    br = -ar * right_startend[0] + (120 - y[right_startend[0]]);
    
    
    
    float x1, x2;
    x1 = (120 - bl)/al;
    x2 = (120 - br)/ar;
    
    if ( countnull > 315){
        printf("steering : case 0 : unfound \n ");
        //steer = (*tomsteer)*2/3 +(midsteer)/3;
        steer = (*tomsteer + midsteer)/2;
        
    }
    else if ( al > 1 && ar >1){
        //steer = (*tomsteer)*2/3 +(midsteer)/3;
        steer = (*tomsteer + midsteer)/2;
        printf("steering : case 0 : unfound \n");
    }
    /* Hoon's world
    else if ( 80 <= diff0 && diff0<=200){
        // Check : Is hill?
        if( (0.7<= *l && *l<=1.0) && (-1.0 <= *r && *r<= -0.7) ){
            int factor = 3;
            printf("Hoon's #$ left : %d  , right : %d    , leftT : %f  , rightT :  %f \n\n", left_startend[0] , right_startend[0], *l , *r);
            
            if( abs(diff0 - 160 ) >=80 ){
                factor = 5;
            }
            // to left
            if( diff0/2 <=160  ){
                steer = steer + diff0*factor/2;
            }
            // to right
            else{
                steer = steer - diff0;
            }
            
        }
    }*/
    
    
    
    
    
    /*	else if(right_startend[1]!= left_startend[1] &&
     60<= right_startend[1] - left_startend[1] && right_startend[1]-left_startend[1]<=180)
     {
     if( right_startend[0] != left_startend[0] &&
     100 <= right_startend[0] - left_startend[0] &&
     right_startend[0] - left_startend[0] <= 200){
     
     printf("!@#$ ~~~ %d ~~~ %d \n  ", right_startend[0] , left_startend[0]);
     steer = midsteer + (160 - (right_startend[0] + left_startend[0])/2)*4;
     
     }
     }
     else if( right_startend[0] != left_startend[0] &&
	    100 <= right_startend[0] - left_startend[0] &&
	    right_startend[0] - left_startend[0] <= 200)
     {
     int l60, l80, r60, r80;
     int i;
     
     for(i=0; i<320; i++)
     if(isLine(&bimgs, i, 240))
     {
     l80 = i;
     break;
     }
     
     for(i=0; i<320; i++)
     if(isLine(&bimgs, i, 260))
     {
     l60 = i;
     break;
     }
     
     for(i=319; i>=0; i--)
     if(isLine(&bimgs, i, 240))
     {
     r80 = i;
     break;
     }
     
     for(i=319; i>=0; i--)
     if(isLine(&bimgs, i, 260))
     {
     r60 = i;
     break;
     }
     
     printf("!@#$ ~~~ %d ~~~ %d ~~~ %d ~~~ %d \n  ", l60, l80, r80, r60);
     steer = midsteer + (160 - (l60+l80+r80+r60)/4)*4;
     }
     */
    
    else if (  al > 0.5 && ar < -0.5){
        steer = midsteer - ( (x1 + x2)/2 - 160);
        if (steer > midsteer + straightsteer_threshold)
            steer = midsteer +straightsteer_threshold;
        if (steer < midsteer - straightsteer_threshold)
            steer = midsteer - straightsteer_threshold;
        printf("steering : case 1 : straight \n");
    }
    
    else if( ( y[left_startend[0]] >115 || y[right_startend[0]] > 115 ) && ( (al >0.5 && al<1) || (ar>0.5 && ar <1) ) ){
        if (y[left_startend[0]] >115 )
            steer = 1300 - left_startend[0];
        else
            steer = 1300 - right_startend[0];
        printf("steering : case 2 (%d , %d): sudden steer when straight \n",y[left_startend[0]],y[right_startend[0]]);
    }
    
    else if ((y[left_startend[1]] >115 || y[right_startend[1]] >115) && ( (al <-0.5) || (ar<-0.5))){
        if(y[left_startend[1]]>115 )
            steer = 1700 + (320 - left_startend[1]);
        
        else
            steer = 1700 + (320 - right_startend[1]);
        printf("steering : case 3 (%d , %d): sudden steer when straight \n",y[left_startend[1]],y[right_startend[1]]);
    }
    
    else if( ( y[left_startend[0]] >115 || y[right_startend[0]] > 115 ) && ( (al<0.5 &&al >0) || (ar>0 &&ar<0.5)) ){
        if (y[left_startend[0]] >115 )
            steer = 1100 - left_startend[0];
        else
            steer = 1100 - right_startend[0];
        printf("steering : case 4 (%d , %d): sudden steer when curve \n",y[left_startend[0]],y[right_startend[0]]);
    }
    
    else if ((y[left_startend[1]] >115 || y[right_startend[1]] >115) && ( (al>-0.5 && al <0) || (ar<0 && ar>-0.5))){
        if(y[left_startend[1]]>115 )
            steer = 1900 + (320 - left_startend[1]);
        
        else
            steer = 1900 + (320 - right_startend[1]);
        printf("steering : case 5 (%d , %d): sudden steer when curve \n",y[left_startend[1]],y[right_startend[1]]);
    }
    
    else if(y[left_startend[0]] > turn_threshold && (left_startend[1]-left_startend[0]) > 80 &&al < 0.5 && al > 0){
        steer = midsteer -(int)x1+100-160;;
        printf("steering : case 6 : turning right at curve \n");
        //printf("left_startend %d,%d",y[left_startend[0]],y[left_startend[1]]);
    }
    else if(y[right_startend[1]] > turn_threshold && (right_startend[1]-right_startend[0]) > 80 &&ar > -0.5 && ar < 0){
        
        steer = midsteer -((int)x2 -320) + 200 - 160 ;
        printf("steering : case 7 : turning left at curve \n");
        //printf("right_startend %d,%d",y[right_startend[0]],y[right_startend[1]]);
    }
    
    //if only one line detected
    else if ( (al !=1000 && ar == 1000 )|| (al !=1000 && al > 0.5 && ar > -0.5) ){
        steer = midsteer -(int)x1+200-160;
        printf("steering : case 8 : only left line detected \n");
    }
    
    else if ( (al == 1000 && ar != 1000) || (ar < -0.5 && al < 0.5 && ar != 1000) ){
        steer = midsteer -((int)x2 -320) + 100 - 160 ;
        printf("steering : case 9 : only right line detected \n");
    }
    
    else {
        //steer = (*tomsteer)*2/3 +(midsteer)/3;
        steer = (*tomsteer + midsteer)/2;
        printf("steering : case null ");
    }
    
    
    
    if (steer > 2000)
        steer = 2000;
    if (steer < 1000)
        steer = 1000;
    
    *tomsteer = steer;
}

void moveSpeed(int angle, int speed){
    SteeringServoControl_Write(angle);
    PositionControlOnOff_Write(UNCONTROL);
    SpeedControlOnOff_Write(CONTROL);
    DesireSpeed_Write(speed);
}



/*
10/16 Edit
흰색 정지선과 점선을 구분하는 함수
영상에서 가로를 쫙 훑으면서 
일정 길이의 흰색, 검은색, 흰색을 만나는지에 따라서 구분..

*/
int white_line(IplImage *img, int* dashline_on, int* stopline_on)
{//0
    int h = img->height;
    int w = img->width;
    int d = img->nChannels;
    int step = 8;
    unsigned char R,G,B;
    int countWhite1 = 0;
    int countWhite2 = 0;
    int countBlack = 0;
    int chunk = 0;
    int i, j;
    
    //go when your dashline_on 0, -1
    if(*dashline_on != 0 && *dashline_on != -1)
        return 0;
    if(*stopline_on == 1 || *stopline_on == 2)
        return 0;


if(*stopline_on >= 3 && *stopline_on <= 5){
	for(j = w/2; j < w; j++){
		for(i = h-1; i>= h/2; i--){

 	    B = (uchar)img->imageData[d*w*i + d*j + 0];
            if (B > 200){//3
        	  countWhite1 ++;
                  if(countWhite1 > 100){//5
                        printf("result : outofrotary\n");
                        *stopline_on += 1;
                        return 0;
		  }//5
	     }//3

		}//fori
	}//forj
}//stopline 
 else{
    for(i = h-1; i>= h/2; i--){//1
        if (chunk > 1){
            printf("result : dashline\n");
            *dashline_on = 1;
            return 0;
        }
       //printf("%d %d %d\n", countWhite1, countWhite2, countBlack);
        countWhite1 = 0, countWhite2 = 0;
        countBlack = 0;
        for(j = 0; j < w; j++){//2
            //RGB same = 255 or 0; so check one
            B = (uchar)img->imageData[d*w*i + d*j + 0];
            if (B > 200){//3
        	    if(countBlack == 0){//4
                    	countWhite1 ++;
                    if(countWhite1 > 200){//5
                        printf("result : stopline\n");
                        *stopline_on = 1;
                        return 0;
                    }//5
            }//4
       	    else
            	countWhite2 ++;
       	    }//3
            else if(countWhite1 != 0 && countWhite2 == 0){ //B == 0 and
                countBlack++;
            }
        }//2

    //divide chunk
        if(countBlack > 180 && countWhite1 > 5 && countWhite2 > 5 && countWhite1 < 20 && countWhite2 < 20){
        printf("%d %d %d\n", countBlack, countWhite1, countWhite2);
                chunk++;
        }
    }//1
}
    *dashline_on = 0, *stopline_on = 0;
    return 0;
}//0 


//VERTICAL PARKING
void parkV(void)
{
    int steer = 1510;
    int behind, right_rear,left_rear, right_front,left_front =0;
    int c1,c2,c3 =0;
    
    // after 2nd wall , aju jjo  geum toward
    moveSpeed(steer,80);
    usleep(250000);
    moveSpeed(steer,0);
    
    // 1st backward movement
    moveSpeed(1030,-130);
    while(1){
        behind = DistanceSensor(4);
        right_rear = DistanceSensor(3);
        left_rear = DistanceSensor(5);
        //printf("1:  behind : %d     , right_rear :  %d \n", behind, right_rear);
        
        
        // behind <= 400 || right_rear <= 400  --> waste value (at first time)
        if( behind > 400 && right_rear > 400 ){
            
            // 1. if right side is too close at 2nd wall
            if( right_rear >=2700){
                
                //to debug
                printf("right_rear >= 2500 \n");
                
                // have to go forward with left steer until right_rear value is under 400
                // right_rear < 400 is can not detect 2nd wall case
                moveSpeed(1900,100);
                while(DistanceSensor(3)>400){}
                
                //after left steer , have to go forward with right steer few msec
                //usleep time is associated with how mush close at 2nd wall and right_rear value
                moveSpeed(1100,100);
                usleep(500000*(right_rear/1000));
                
                //case 1 flag
                c1=1;
                
                //to debug
                printf("D2   behind : %d     , right_rear :  %d , left_rear  : %d\n", behind, right_rear,left_rear);
                break;
            }
            
            // 2. if left side is too close at 1st wall
            if( left_rear >= 2700){
                
                //to debug
                printf("left_rear >= 2500 \n");
                
                
                // have to go forward with right steer until left_rear value is under 400
                // left_rear < 400 is can not detect 1st wall case
                moveSpeed(1100,100);
                while(DistanceSensor(5)>400){}
                
                
                //after right steer , have to go forward with left steer few msec
                //usleep time is associated with how mush close at 1st wall and left_rear value
                moveSpeed(1900,100);
                usleep(250000*(left_rear/1000));
                
                //case 2 flag
                c2 =1;
                
                //to debug
                printf("D3   behind : %d     , right_rear :  %d , left_rear  : %d\n", behind, right_rear,left_rear);
                break;
                
            }
            
            // 3. if behind is too close at 1st wall
            if( behind>=1200){
                
                //case 3 flag
                c3 =1;
                
                //to debug
                printf("behind>=1200,,,,,, %d \n", behind);
                
                // have to go forward with left steer few msec
                moveSpeed(1700,100);
                usleep(300000*(behind+right_rear+1000)/1000);
                
                //to debug
                printf("D4   behind : %d     , right_rear :  %d , left_rear  : %d\n", behind, right_rear,left_rear);
                break;
            }
            
            // 4. it's good
            // one shot
            if ( (2000 <= right_rear && right_rear<2700) && (2000 <= left_rear && left_rear<2700) ){
                printf("DC#########\n");
                
                printf("D4   behind : %d     , right_rear :  %d , left_rear  : %d\n", behind, right_rear,left_rear);
                break;
            }
            
        }
    }
    //return 0;
    
    // 2nd backward movement
    if( c1)
        moveSpeed(1400,-100);
    else if (c2)
        moveSpeed(1600,-100);
    else if (c3)
        moveSpeed(steer,-100);
    else
        moveSpeed(steer,-100);
    
    printf("D5 in\n");
    
    
    
    // have to???
    while(1){
        right_rear = DistanceSensor(3);
        left_rear= DistanceSensor(5);
        
        if( left_rear >= 1800 && right_rear>=1800){
            printf("D5:  right_rear :  %d , left_rear  : %d\n",right_rear,left_rear);
            moveSpeed(steer,0);
            break;
        }
    }
    ////////////////////
    
    printf("D6 in\n");
    
    
    
    //3rd backward movement
    while(1){
        int msteer =1510;
        
        behind = DistanceSensor(4);
        right_rear = DistanceSensor(3);
        left_rear= DistanceSensor(5);
        
        //printf("D6:  behind : %d     , right_rear :  %d , left_rear  : %d\n", behind, right_rear,left_rear);
        
        //stop condition
        while(behind < 2000){
            
            
            //if left and right front side is inside at parking lot
            //much important then rear side.... I think
            if( left_front >=2000 && right_front>= 2000){
                msteer = steer + (left_front - right_front)/5;
                msteer >=1900 ? 1900 : msteer <= 1100 ? 1100 : msteer ;
                
                moveSpeed( msteer , -90);
                
            }
            else{
                
                //if right or left rear side is too close at any wall, then steer value is midsteer
                if( right_rear >=3500 || left_rear >=3500){
                    moveSpeed( steer , -80);
                    
                }
                else
                    moveSpeed( steer + (right_rear-left_rear)/7, -90);
            }
            
            behind = DistanceSensor(4);
            right_rear = DistanceSensor(3);
            left_rear= DistanceSensor(5);
            right_front = DistanceSensor(2);
            left_front = DistanceSensor(6);
            
            
            //to debug
            printf("D7:  behind : %d     , right_rear :  %d , left_rear  : %d \n", behind, right_rear,left_rear);
            
        }
        
        printf(" Parking Success ? \n");
        moveSpeed(steer,0);
        break;
    }
    sleep(3);
    moveSpeed( 1510, 100);
    usleep(500000);
    
    moveSpeed(1100, 150);
    while(1){
        if (DistanceSensor(5)<=400 && DistanceSensor(3) >400)
            break;
    }
    usleep(500000);
    
    return 0;
    
}
//HORIZONTAL PARKING
void parkH(void){
    int steer = 1510;
    int front,behind, right_rear, right_front, left_rear =0;
    //int pre_Rrear = 0;
    int c1,c2,c3,c4 =0;
    
    printf(" ^^^^^^^^^^^ PARK H IN ^^^^^^^^^^^^^^^^\n\n");
    
    // move forward with cur Speed, & midSteer );
    moveSpeed( steer, DesireSpeed_Read());
    usleep(200000);
    
    moveSpeed(steer ,0);
    
    moveSpeed( 1050, -80);
    while( 1 ){
        
        //if( DistanceSensor(4) >= 600)
        if( DistanceSensor(3)>=1300 && DistanceSensor(4)>=600 )
            break;
    }
    moveSpeed(steer,100);
    usleep(150000);
    
    moveSpeed(1990,-80);
    while(DistanceSensor(4)<=2500){}
    
    if( abs(DistanceSensor(2) - DistanceSensor(3)) >= 700){
        moveSpeed(1100,80);
        while(DistanceSensor(1)<=2500){}
        
        moveSpeed(1990,-80);
        while(DistanceSensor(4)<=3700){}
    }
    
    
    moveSpeed(steer,0);
    
    Alarm_Write(ON);
    
    sleep(1);
    
    moveSpeed(1990,80);
    while(1){
        if(DistanceSensor(2)>=3000 || DistanceSensor(1)>=3000 || DistanceSensor(1)<=600){
            break;
        }
    }
    
    moveSpeed(1200,-80);
    while(1){
        if( DistanceSensor(5)>=2000 || DistanceSensor(4)>=2000)
            break;
    }
    moveSpeed(1990,80);
    while(DistanceSensor(2)>=600){
    }
    moveSpeed(1100,150);
    usleep(500000);
    
    //	moveSpeed(1700,80);
    //	while(DistanceSensor(2)>1000){
    //	}
    //	moveSpeed(1300,150);
    //	usleep(500000);
    
    return 0;
    //H1
    
}

void SensorCheck(){
    printf(" Front : %d     right_front : %d    right_rear : %d    Behind : %d     left_rear : %d\n\n", DistanceSensor(1),DistanceSensor(2),DistanceSensor(3), DistanceSensor(4),DistanceSensor(5));
    usleep(500000);
}



void *ControlThread(void *unused)
{
    int i=0;
    char fileName[30];
    NvMediaTime pt1 ={0}, pt2 = {0};
    NvU64 ptime1, ptime2;
    struct timespec;
    
    //NEW PARAMETER
    int steer_y[400];
    int hill_y[400];
    int bump_y[400];
    int bump2[400];
    int final[400];
    int cnt_final = 0;
    
    //before 1510
    int midsteer = 1510;
    int tomsteer=midsteer;
    
    int speed = 100;
    unsigned char status;
    int red_stop =0;
    int red_on = 0;
    int cur_reddetect = 0;
    int pre_reddetect = 0;
    
    int bump_on=1;
    
    int curr_dash = 0;
    int prev_dash = 0;
    
    int dashline_on = 0;
    int stopline_on = 0;
    
    //PARKING PARAMETER
    int ppre, preDis , curDis ;
    int dflag = 0;
    int fcount = 0, fcount2 =0;
    int parkdetect = 0;
    
    // INIT
    CarControlInit();
    
    sleep(1);
    
    CameraXServoControl_Write(1480);
    CameraYServoControl_Write(1700);
    SteeringServoControl_Write(midsteer);
    
    sleep(1);
    
    PositionControlOnOff_Write(UNCONTROL);
    SpeedControlOnOff_Write(CONTROL);
    DesireSpeed_Write(150);
    //return 0;
    
    /*
     moveSpeed(1010,-130);
     usleep(800000);
     while(DistanceSensor(4)<3000){}
     */
    /*
     parkV();
     return 0;
     moveSpeed(midsteer,0);
     for(i=0;  i< 10 ; i++){
     printf(" right -- > %d    , behind --> %d \n\n",DistanceSensor(3), DistanceSensor(4));
     usleep(500000);
     
     }
     
     return 0;
     */
    ppre = 0, preDis = 0;
    curDis = DistanceSensor(3);
    
    
    
    //------------------------------------------------------------
    
    
    
    // cvCreateImage
    IplImage* imgOrigin;
    //IplImage* imgCanny;
    
    // cvCreateImage
    imgOrigin = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);
    //imgCanny = cvCreateImage(cvGetSize(imgOrigin), IPL_DEPTH_8U, 1);
    
    while(1)
    {
        pthread_mutex_lock(&mutex);
        pthread_cond_wait(&cond, &mutex);
        
        
        GetTime(&pt1);
        ptime1 = (NvU64)pt1.tv_sec * 1000000000LL + (NvU64)pt1.tv_nsec;
        
        
        Frame2Ipl(imgOrigin); // save image to IplImage structure & resize image from 720x480 to 320x240
        pthread_mutex_unlock(&mutex);
        
        /*
         printf("2 >> %d , 3 >> %d , 4 >> %d , 5 >> %d , 6 >> %d \n", DistanceSensor(2) , DistanceSensor(3) , DistanceSensor(4) , DistanceSensor(5) , DistanceSensor(6) );
         continue;
         */
        //cvCanny(imgOrigin, imgCanny, 100, 100, 3);
        
        //sprintf(fileName, "captureImage/imgCanny%d.png", i);
        //cvSaveImage(fileName , imgCanny, 0);
        
        sprintf(fileName, "captureImage/imgOrigin%d.png", i);
        cvSaveImage(fileName, imgOrigin, 0);
        
        //continue
        // TODO : control steering angle based on captured image ---------------
        
        //printf(" right : %d  , behind : %d , left :%d \n", DistanceSensor(3),DistanceSensor(4),DistanceSensor(5));
        
        init_array(steer_y, 400, 1000);
        init_array(hill_y, 400, 1000);
        init_array(bump_y, 400, 1000);
        init_array(bump2, 400, 0);
        init_array(final, 400, 1000);
        
        
        // imgOrigin -- > binary_image
        IplImage **bimgs = binary_image_array(imgOrigin, steer_y, hill_y, bump_y, bump2, final,&cnt_final, &dashline_on, &stopline_on);
        
        
        
        sprintf(fileName, "captureImage/%d.png", i);
        cvSaveImage(fileName,imgOrigin ,0);
        
        sprintf(fileName, "captureImage/0_%d.png", i);
        cvSaveImage(fileName,bimgs[0] ,0);
        
        sprintf(fileName, "captureImage/1_%d.png", i);
        cvSaveImage(fileName,bimgs[1] ,0);
        
        
        sprintf(fileName, "captureImage/2_%d.png", i);
        cvSaveImage(fileName,bimgs[2] ,0);
        
        
        int num_y = 320;
        float left_tangent, right_tangent;
        int left_startend[2] = {0,0};
        int right_startend[2] = {0,0};
        
        checkline(steer_y,num_y,left_startend,right_startend,&left_tangent,&right_tangent);
        
        
        if (checkbump(bump2,num_y,&tomsteer,midsteer,bump_on)==0){
            checkdir(steer_y,left_startend,right_startend,&left_tangent,&right_tangent,&tomsteer,midsteer, bimgs[0]);
        }
        
        SteeringServoControl_Write(tomsteer);
        
        //PARK
        ppre = preDis;
        preDis = curDis;
        curDis = DistanceSensor(3);
        if ( dflag == 0 ){
            // 450 --> 300
            if(curDis - ppre >= 400){
                printf("#####first rising#####\n");
                dflag = 1;
                fcount = i;
                //Timer start
            }
        }
        else if ( dflag == 1){
            //have to fall in 5 frame
            if( i - fcount > 5) {dflag = 0; printf("$$$$$reset$$$$$\n");} //timer reset
            else{
                // 450 --> 300
                if(ppre - curDis >= 400) {
                    dflag = 2;
                    printf("#####first falling######\n");
                    //Timer Start
                    fcount2 = i;
                }
            }
        }
        else if ( dflag == 2){
            if(i - fcount2 > 10) {dflag = 0; printf("$$$ohhae$$$$\n");}
            else{
                // 450 --> 300
                if(curDis - ppre >= 450){
                    //--> parking start
                    printf("##################parking   start\n");
                    parkdetect = i - fcount;
                    
                    // hoon edit 20 --> 15
                    if( 2 * parkdetect >= 20){
                        
                        printf("parkH STOP~P~P~PPP~P\n");
                        parkH();
                        
                    }
                    else {
                        printf("parkV STOP~P~P~P~P~P~P~P~P\n");
                        parkV();
                        
                    }
                    
                    
                    printf("##################parking   end\n");
                    dflag = 0;
                    
                }
            }
        }
        
        //RED DETECT SOURCE
        if(red_on == 1){ //original - 0
            cur_reddetect = red_detect(imgOrigin, &red_stop);
            //first
            if ( cur_reddetect == 1 && pre_reddetect == 0 && red_stop == 1){
                //speed = DesireSpeed_Read();
                DesireSpeed_Write(0);
                pre_reddetect = 1;
                usleep(3000000);
                tomsteer = midsteer;
            }
            //
            else if ( cur_reddetect == 1 && pre_reddetect == 1 ){
                usleep(400000);
                tomsteer = midsteer;
            }
            //the end
            else if ( cur_reddetect == 0 && pre_reddetect == 1 && red_stop == 0){
                DesireSpeed_Write(140); //original : 140
                pre_reddetect = 0;
                red_on = 1;
                //bump_on = 2;
            }
        }
        
        
        //DASH LINE CHECK FOR OVERTAKING SECTION
        white_line(bimgs[2], &dashline_on, &stopline_on);
        
        if(dashline_on >= 10){
            printf("%$%$%$%$%&^&^&^&^ turnturnturn \n");
            moveSpeed(midsteer - 400 * (dashline_on - 11), 150);
            Winker_Write(ALL_ON);
            usleep(1000000);
            Winker_Write(ALL_OFF);
            dashline_on *= -1;
        }

	//ROTARY STOPLINE DETECT
	if(dashline_on == 0 && stopline_on == 1){
		printf("stop and watch");
		moveSpeed(1510, 0);
		stopline_on = 2;
	}
	
	//TRAFFIC LIGHT        
        if(dashline_on == -1 && stopline_on == 1){
            stopline_on == 0;
            moveSpeed(1510, 0);
            usleep(100000); 
        }
        
        // ---------------------------------------------------------------------
        
        GetTime(&pt2);
        ptime2 = (NvU64)pt2.tv_sec * 1000000000LL + (NvU64)pt2.tv_nsec;
        printf("%d------------------------------operation time=%llu.%09llu[s]\n",i, (ptime2-ptime1)/1000000000LL, (ptime2-ptime1)%1000000000LL);  
        
        
        i++;
    }
    
}

int main(int argc, char *argv[])
{
    int err = -1;
    TestArgs testArgs;
    
    
    //BBC 
    frame = atoi(argv[2]);
    printf (" FRAME ---->  %d \n\n ", frame); 
    // BBC END
    
    CaptureInputHandle handle;
    
    NvMediaVideoCapture *vipCapture = NULL;
    NvMediaDevice *device = NULL;
    NvMediaVideoMixer *vipMixer = NULL;
    NvMediaVideoOutput *vipOutput[2] = {NULL, NULL};
    NvMediaVideoOutput *nullOutputList[1] = {NULL};
    FILE *vipFile = NULL;
    
    NvSemaphore *vipStartSem = NULL, *vipDoneSem = NULL;
    NvThread *vipThread = NULL;
    
    CaptureContext vipCtx;
    NvMediaBool deviceEnabled = NVMEDIA_FALSE;
    unsigned int displayId;
    
    pthread_t cntThread;
    
    signal(SIGINT, SignalHandler);
    
    memset(&testArgs, 0, sizeof(TestArgs));
    if(!ParseOptions(argc, argv, &testArgs))
        return -1;
    
    printf("1. Create NvMedia capture \n");
    // Create NvMedia capture(s)
    switch (testArgs.vipDeviceInUse)
    {
        case AnalogDevices_ADV7180:
            break;
        case AnalogDevices_ADV7182:
        {
            CaptureInputConfigParams params;
            
            params.width = testArgs.vipInputWidth;
            params.height = testArgs.vipInputHeight;
            params.vip.std = testArgs.vipInputtVideoStd;
            
            if(testutil_capture_input_open(testArgs.i2cDevice, testArgs.vipDeviceInUse, NVMEDIA_TRUE, &handle) < 0)
            {
                MESSAGE_PRINTF("Failed to open VIP device\n");
                goto fail;
            }
            
            if(testutil_capture_input_configure(handle, &params) < 0)
            {
                MESSAGE_PRINTF("Failed to configure VIP device\n");
                goto fail;
            }
            
            break;
        }
        default:
            MESSAGE_PRINTF("Bad VIP device\n");
            goto fail;
    }
    
    
    if(!(vipCapture = NvMediaVideoCaptureCreate(testArgs.vipInputtVideoStd, // interfaceFormat
                                                NULL, // settings
                                                VIP_BUFFER_SIZE)))// numBuffers
    {
        MESSAGE_PRINTF("NvMediaVideoCaptureCreate() failed for vipCapture\n");
        goto fail;
    }
    
    
    printf("2. Create NvMedia device \n");
    // Create NvMedia device
    if(!(device = NvMediaDeviceCreate()))
    {
        MESSAGE_PRINTF("NvMediaDeviceCreate() failed\n");
        goto fail;
    }
    
    printf("3. Create NvMedia mixer(s) and output(s) and bind them \n");
    // Create NvMedia mixer(s) and output(s) and bind them
    unsigned int features = 0;
    
    
    features |= NVMEDIA_VIDEO_MIXER_FEATURE_VIDEO_SURFACE_TYPE_YV16X2;
    features |= NVMEDIA_VIDEO_MIXER_FEATURE_PRIMARY_VIDEO_DEINTERLACING; // Bob the 16x2 format by default
    if(testArgs.vipOutputType != NvMediaVideoOutputType_OverlayYUV)
        features |= NVMEDIA_VIDEO_MIXER_FEATURE_DVD_MIXING_MODE;
    
    if(!(vipMixer = NvMediaVideoMixerCreate(device, // device
                                            testArgs.vipMixerWidth, // mixerWidth
                                            testArgs.vipMixerHeight, // mixerHeight
                                            testArgs.vipAspectRatio, //sourceAspectRatio
                                            testArgs.vipInputWidth, // primaryVideoWidth
                                            testArgs.vipInputHeight, // primaryVideoHeight
                                            0, // secondaryVideoWidth
                                            0, // secondaryVideoHeight
                                            0, // graphics0Width
                                            0, // graphics0Height
                                            0, // graphics1Width
                                            0, // graphics1Height
                                            features , // features
                                            nullOutputList))) // outputList
    {
        MESSAGE_PRINTF("NvMediaVideoMixerCreate() failed for vipMixer\n");
        goto fail;
    }
    
    printf("4. Check that the device is enabled (initialized) \n");
    // Check that the device is enabled (initialized)
    CheckDisplayDevice(
                       testArgs.vipOutputDevice[0],
                       &deviceEnabled,
                       &displayId);
    
    if((vipOutput[0] = NvMediaVideoOutputCreate(testArgs.vipOutputType, // outputType
                                                testArgs.vipOutputDevice[0], // outputDevice
                                                NULL, // outputPreference
                                                deviceEnabled, // alreadyCreated
                                                displayId, // displayId
                                                NULL))) // displayHandle
    {
        if(NvMediaVideoMixerBindOutput(vipMixer, vipOutput[0], NVMEDIA_OUTPUT_DEVICE_0) != NVMEDIA_STATUS_OK)
        {
            MESSAGE_PRINTF("Failed to bind VIP output to mixer\n");
            goto fail;
        }
    }
    else
    {
        MESSAGE_PRINTF("NvMediaVideoOutputCreate() failed for vipOutput\n");
        goto fail;
    }
    
    
    
    printf("5. Open output file(s) \n");
    // Open output file(s)
    if(testArgs.vipFileDumpEnabled)
    {
        vipFile = fopen(testArgs.vipOutputFileName, "w");
        if(!vipFile || ferror(vipFile))
        {
            MESSAGE_PRINTF("Error opening output file for VIP\n");
            goto fail;
        }
    }
    
    printf("6. Create vip pool(s), queue(s), fetch threads and stream start/done semaphores \n");
    // Create vip pool(s), queue(s), fetch threads and stream start/done semaphores
    if(NvSemaphoreCreate(&vipStartSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipStartSem\n");
        goto fail;
    }
    
    if(NvSemaphoreCreate(&vipDoneSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipDoneSem\n");
        goto fail;
    }
    
    vipCtx.name = VIP_NAME;
    
    vipCtx.semStart = vipStartSem;
    vipCtx.semDone = vipDoneSem;
    
    vipCtx.capture = vipCapture;
    vipCtx.mixer = vipMixer;
    vipCtx.fout = vipFile;
    
    vipCtx.inputWidth = testArgs.vipInputWidth;
    vipCtx.inputHeight = testArgs.vipInputHeight;
    
    vipCtx.timeout = VIP_FRAME_TIMEOUT_MS;
    
    vipCtx.displayEnabled = testArgs.vipDisplayEnabled;
    vipCtx.fileDumpEnabled = testArgs.vipFileDumpEnabled;
    
    if(testArgs.vipCaptureTime)
    {
        vipCtx.timeNotCount = NVMEDIA_TRUE;
        vipCtx.last = testArgs.vipCaptureTime;
    }
    else
    {
        vipCtx.timeNotCount = NVMEDIA_FALSE;
        vipCtx.last = testArgs.vipCaptureCount;
    }
    
    
    if(NvThreadCreate(&vipThread, CaptureThread, &vipCtx, NV_THREAD_PRIORITY_NORMAL) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvThreadCreate() failed for vipThread\n");
        goto fail;
    }
    
    printf("wait for ADV7182 ... one second\n");
    sleep(1);
    
    printf("7. Kickoff \n");
    // Kickoff
    NvMediaVideoCaptureStart(vipCapture);
    NvSemaphoreIncrement(vipStartSem);
    
    printf("8. Control Thread\n");
    pthread_create(&cntThread, NULL, &ControlThread, NULL); 
    
    printf("9. Wait for completion \n");
    // Wait for completion
    NvSemaphoreDecrement(vipDoneSem, NV_TIMEOUT_INFINITE);
    
    
    err = 0;
    
fail: // Run down sequence
    // Destroy vip threads and stream start/done semaphores
    if(vipThread)
        NvThreadDestroy(vipThread);
    if(vipDoneSem)
        NvSemaphoreDestroy(vipDoneSem);
    if(vipStartSem)
        NvSemaphoreDestroy(vipStartSem);
    
    printf("10. Close output file(s) \n");
    moveSpeed(1500, 0);
    // Close output file(s)
    if(vipFile){
        fclose(vipFile);
        
    }
    
    // Unbind NvMedia mixer(s) and output(s) and destroy them
    if(vipOutput[0])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[0], NULL);
        NvMediaVideoOutputDestroy(vipOutput[0]);
    }
    if(vipOutput[1])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[1], NULL);
        NvMediaVideoOutputDestroy(vipOutput[1]);
    }
    if(vipMixer)
        NvMediaVideoMixerDestroy(vipMixer);
    
    
    // Destroy NvMedia device
    if(device)
        NvMediaDeviceDestroy(device);
    
    // Destroy NvMedia capture(s)
    if(vipCapture)
    {
        NvMediaVideoCaptureDestroy(vipCapture);
        
        // Reset VIP settings of the board
        switch (testArgs.vipDeviceInUse)
        {
            case AnalogDevices_ADV7180: // TBD
                break;
            case AnalogDevices_ADV7182: // TBD
                //testutil_capture_input_close(handle);
                break;
            default:
                break;
        }
    }
    
    return err;
}

