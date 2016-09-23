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

#define DISTANCE_SENSOR
static short frame = 0;
int i = 0;

// STATICPAR

int beforePos0 = 0;
int beforePos1 = 0;
int gogo = 0;
//rotarypar
int rotaryFlag = 0;
int rotarySpeed = 120;
int stopline_on = 0;
int newspeed = 220;

int tfl_on = 0;

//parkpar
int parkHflag = 0;
int parkVflag = 0;

int ggoom = 0;

//overpar
int st_turnback =0;
int overTakingFlag = 0;

//hillpar
int hillFlag = 0;
int lowerBound = 320;
int upperBound = 0;
int hillSpeed = 150;

//redpar

//tflpar
int tflCenter = 0;

//etcpar












//-----------------------------------------------------------------------------------

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


//TFL FUNCTION//
void tfl(IplImage *image, int * tfl_on, int * tfl_direction,int * max_hill_threshold)
{                

	IplImage *imagegray = 0;         
	IplImage *cropped = 0;          
	char filename[100];
	int ii = 0;
	char* data;
	unsigned char r,g,b;              

	IplImage *roi = cvCreateImage(cvSize(320,120), IPL_DEPTH_8U, 3);
	cvSetImageROI(image, cvRect(0,0,320,120));
	cvCopy(image, roi, NULL);
	cvResetImageROI(image);
	IplImage *gray = cvCreateImage(cvGetSize(roi), IPL_DEPTH_8U, 1);
	cvCvtColor(roi,gray,CV_RGB2GRAY);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* results = 0;		
	results = cvHoughCircles(gray, storage, CV_HOUGH_GRADIENT, 1, 20,100,20,8,15);

	int i;
	int tfl_i;       
	int traf_x[10];
	int traf_y[10];
	int traf_rd[10];
	int traf_r[10];
	int traf_g[10];
	int traf_b[10];
	int findb;           
	int bmax;
	bmax = 0;
	findb = 60;
	tfl_i = 0;

	printf("circles : %d\n", results->total);
	for ( i = 0; i < results->total; i++)
	{
		float* p = (float*)cvGetSeqElem(results, i);
		CvPoint pt = cvPoint( cvRound(p[0]), cvRound(p[1]) );
		cvCircle(cropped, pt, cvRound(p[2]), CV_RGB(0xff, 0xff, 0xff), 1,8,0);
		printf("rd : %d, x : %d , y : %d\n",cvRound(p[2]), pt.x, pt.y);
		tflCenter += pt.x;
		traf_x[i] = pt.x;
		traf_y[i] = pt.y;
		traf_rd[i] = cvRound(p[2]);

		data=image->imageData;
		traf_b[i]=data[3*((image->width)*(pt.y)+pt.x)+0];
		traf_g[i]=data[3*((image->width)*(pt.y)+pt.x)+1];
		traf_r[i]=data[3*((image->width)*(pt.y)+pt.x)+2];
		printf("%d,%d,%d\n",traf_r[i], traf_g[i], traf_b[i]);

		if (bmax <=traf_b[i]){bmax = traf_b[i]; tfl_i = i;}
	}

	printf("traf_r[tfl_i] = %d /n",traf_r[tfl_i]);
	tflCenter /= results->total;

	if(bmax < findb || traf_r[tfl_i] > 40 ){
		printf("\n PLEASE STOP! \n");	
		printf("tfl_stop();\n");

	}

	else if(bmax >= findb){
		printf("\n TURN RIGHT OR LEFT \n");
		printf("rd : %d    pt.x : %d    pt.y : %d   b:  %d  \n", traf_rd[tfl_i], traf_x[tfl_i], traf_y[tfl_i], traf_b[tfl_i]);

		if (data[3*((image->width)*(traf_y[tfl_i]+traf_rd[tfl_i]*3/5)+traf_x[tfl_i]+traf_rd[tfl_i]*1/5)+1] < findb
				&&data[3*((image->width)*(traf_y[tfl_i]-traf_rd[tfl_i]*3/5)+traf_x[tfl_i]+traf_rd[tfl_i]*1/5)+1] < findb )
		{ printf("TURN LEFT \n");

			*tfl_on = 2;
			*tfl_direction = 1;
			*max_hill_threshold = (-185 + traf_x[tfl_i])/10;
			if( *max_hill_threshold > 15)
				*max_hill_threshold = 15;
			if( *max_hill_threshold < -15)
				*max_hill_threshold = -15;
			printf(" max_hill_thres : %d   ", *max_hill_threshold);

		}

		else if(data[3*((image->width)*(traf_y[tfl_i]+traf_rd[tfl_i]*3/5)+traf_x[tfl_i]+traf_rd[tfl_i]*1/5)+1] >= findb
				&&data[3*((image->width)*(traf_y[tfl_i]-traf_rd[tfl_i]*3/5)+traf_x[tfl_i]+traf_rd[tfl_i]*1/5)+1] >= findb )
		{ printf("TURN RIGHT \n");


			*tfl_on = 2;
			*tfl_direction = -1;
			*max_hill_threshold = (220 - traf_x[tfl_i])/10;
			if( *max_hill_threshold > 15)
				*max_hill_threshold = 15;
			if( *max_hill_threshold < -15)
				*max_hill_threshold = -15;			

		}		
	}
	else
		tflCenter = 0;

}

int maxhilly ( int * hill_y){
	int iii = 0;
	int max = 0;
	for(iii = 100; iii<200; iii++){
		if( hill_y[iii] > max && hill_y[iii] <200)
			max = hill_y[iii];

	}
	printf("max && %d && ", max);
	return max;

}

//TFL END//

////////// Team.c --> BBC
IplImage** binary_image_array(IplImage *img, int *steer_y, int *hill_y, int *final, int * cnt_final, int * dashline_on, int *stopline_on)
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
	int final_stop = 0;
	int turnback = 0;   
	int pos[3] = {0,};
	int a = 0;
	int posSum =0;

	int dashPoint = 0;
	int leftPoint =0;
	int rightPoint =0;
	int wLine =0;
	int preWLine = 0;
	int preX = 0;
	int isWhiteBlock = 0;
	int numWhiteBlock = 0;


	for( i = 0 ; i < w; i++ ){
		hill_stop = 0;
		hill_stop2 = 0;
		final_stop = 0;


		//if dashline is on or your rotary-> find black
		/*
		   회전교차로 진입했을 때 검은색 픽셀을 찾는 If
		 */
		if(rotaryFlag == 1 || *dashline_on==1/*|| rotaryFlag == 2*/){

			if(i < 50) a = 0;
			else if (i < 270) a = 1;
			else a = 2;

			/*
			   연산의 속도를 위해서 영상의 위아래 어느정도를 자름
			 */
			for(j = h-30; j >= 20; j--){ //can be half
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

			//editwhen 
			/*
			   results[0]은 기본주행을 위해 노란색을 검출한 이미진데 
			   흰색역시 검출이 되기에 평상시에는 흰색을 제거한 노란색을 강조하다가
			   특정 조건에서 흰색을 살려준다 
			   특정 조건 : 
			   1) 회전 교차로 진입 후 탈출을 위한 정지선
			   2) 차로추월진입시 대시라인을 보고난 후 점선을 차선으로 인식 하기 위해
			 */
			if(rotaryFlag == 2 || *dashline_on==1 || *dashline_on==2 ){
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

			if( j ==50){
				results[2]->imageData[d*w*j + d*i + 0] = (uchar)255;
				results[2]->imageData[d*w*j + d*i + 1] = 255;
				results[2]->imageData[d*w*j + d*i + 2] = 255;

			}
			else{
				results[2]->imageData[d*w*j + d*i + 0] = (uchar)gp3;
				results[2]->imageData[d*w*j + d*i + 1] = (uchar)gp3;
				results[2]->imageData[d*w*j + d*i + 2] = (uchar)gp3;
			}
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
	if(rotaryFlag == 1){
		//printf("%d %d\n", beforePos1 - pos[1], beforePos0 - pos[0]);
		gogo = pos[0]+pos[1];
		if(beforePos0 - gogo > 100 && gogo > 1000 && gogo < 2000){
			//usleep(3000000);
			printf("go rotary\n");		
			moveSpeed(1525, rotarySpeed);
			usleep(500000);
			*stopline_on = 0;
			rotaryFlag=2;
			//gogo = 4;
		}
		
		beforePos0 = gogo; 
	}


	if(*dashline_on==2 || tfl_on == 3){
		//gp4 yellow alot -> turn back


		//노란색의 가로선을 찾기위해 
		//특정 길이의 노란색 가로선을 찾는 부분
		//turnback 

		for( ii=0 ; ii< h-30; ii++){

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



			//길이가 150보다 큰 노란색 만나면 복귀 시작

			if( *dashline_on == 2 && turnback > 110 ) {
				st_turnback = turnback;
				if( DistanceSensor(2) < 600){

					moveSpeed(1540, 0);
					usleep(100000);


					//현재는 무조건 차로추월에서 1차선으로만 진행하기때문에
					//오른쪽스티어를 주면서 복귀시작 

					//Sat. Hoon EDIT
					moveSpeed(1000, 150);
					usleep(200000);
			//overTakingFlag =1;
					overTakingFlag = 1;
					*dashline_on=3;
					break;
				}

			}
			if(ii > 110 && tfl_on == 3 && turnback > 120)
			{
				printf("turnnowtraffic\n");
				tfl_on = 4;
			}
				
		}
	
	} 

	//FINDDASH
	if(*dashline_on==0 && hillFlag >= 30)
	{
		for (i = 160 ; i>50; i-=3)
		{
	
			int dashCheck =0;
			for(j=31; j<160; j++)
			{
				B = (uchar)results[2]->imageData[d*w*i + d*j + 0];
				G = (uchar)results[2]->imageData[d*w*i + d*j + 1];
				R = (uchar)results[2]->imageData[d*w*i + d*j + 2];


				if(B==255  && G==255 && R==255)
				{
					wLine++;		
					break;
				}
			}

			for(j=j+1 ; j<160; j++)
			{
				B = (uchar)results[2]->imageData[d*w*i + d*j + 0];
				G = (uchar)results[2]->imageData[d*w*i + d*j + 1];
				R = (uchar)results[2]->imageData[d*w*i + d*j + 2];


				if(B==255&& G==255 && R==255)
					wLine++;
				else
					break;
				

				if(wLine > 15)
					break;
			}
			if( 2 <= wLine && wLine <15)
				leftPoint ++;


			wLine =0;

			for(j=289; j>=160; j--)
			{
				B = (uchar)results[2]->imageData[d*w*i + d*j + 0];
				G = (uchar)results[2]->imageData[d*w*i + d*j + 1];
				R = (uchar)results[2]->imageData[d*w*i + d*j + 2];

				if(B==255 && G==255 && R==255)
				{
					wLine++;
					break;
				}
			}
			for(j=j-1 ; j>=160; j--)
			{
				B = (uchar)results[2]->imageData[d*w*i + d*j + 0];
				G = (uchar)results[2]->imageData[d*w*i + d*j + 1];
				R = (uchar)results[2]->imageData[d*w*i + d*j + 2];


				if(B==255 && G==255 && R==255)
					wLine++;
				else
					break;

				if(wLine > 15)
					break;

			}
			if( 2<= wLine && wLine <15)
				rightPoint++;
		}

	}

	//printf(" left Point : %d      right Point : %d \n" , leftPoint, rightPoint);
	//printf("                                                                   %d\n", dashPoint);
	//if (dashPoint >= 8 && dashPoint <=20)
	if( leftPoint >=10 && rightPoint >=10 )	
		*dashline_on=1;

	if( *dashline_on==1){

		printf(" CAR :%d \n", pos[0] +pos[1]+ pos[2]);
		if(pos[0]+pos[1]+pos[2] > 2700 || ( DistanceSensor(1) > 500 && pos[0]+pos[1]+pos[2] > 2000)  ){
			moveSpeed(1950, 150); //left turn
			Winker_Write(ALL_ON);
			usleep(1200000);
			Winker_Write(ALL_OFF);
			*dashline_on=2;
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
		//cvSaveImage(fileName, img, 0);

		printf("shit!!!!\n");
		*red_stop = 0;
		return 0;
	}
	return 0;
}

int cmp(const void *a, const void *b)
{
	int *x = (int*)a;
	int *y = (int*)b;
	return *x - *y;
}

void checkline(int * y,int num_y, int * left_startend, int * right_startend,float * left_tangent,float * right_tangent)
{

	int ii,jj;
	int findstartend_left = 0;
	int findstartend_right = 0;
	left_startend[0] = 319;


	int bound[4] = {0,};	

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

	if(1 <= hillFlag && hillFlag <= 29)
	{
		bound[0] = left_startend[0] + 120;
		bound[1] = left_startend[1] + 120;
		bound[2] = right_startend[0] + 120;
		bound[3] = right_startend[1] + 120;
	
		qsort(bound, 4, 4, cmp);

		upperBound = bound[0];
		upperBound = upperBound < 0 ? 0 : (upperBound >= 240 ? 239 : upperBound);
		lowerBound = bound[3];
		lowerBound = lowerBound < 0 ? 0 : (lowerBound >= 240 ? 239 : lowerBound);

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

void checkdir(int * y, int * left_startend,int *right_startend,float * l,float * r,int *tomsteer, int midsteer, IplImage *bimgs)
{
	int steer ;//= 1510;
	int countnull = 0, ii;
	int straightsteer_threshold = 80;
	int turnsteer_threshold = 300;
	int turn_threshold = 55;

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
	/*
	   else if (al > 1 && ar > 1){
//steer = (*tomsteer)*2/3 +(midsteer)/3;
steer = (*tomsteer + midsteer)/2;
//printf("steering : case 0 : unfound\n");
printf("                                                                             ?????????\n");
}
	 */
else if (  al > 0.5 && ar < -0.5){
	steer = midsteer - ( (x1 + x2)/2 - 160);
	if (steer > midsteer + straightsteer_threshold)
		steer = midsteer + straightsteer_threshold;
	if (steer < midsteer - straightsteer_threshold)
		steer = midsteer - straightsteer_threshold;
	printf("steering : case 1 : straight \n");
}

else if( ( y[left_startend[0]] >115 || y[right_startend[1]] > 115 ) && ( al > 1 || ar > 1 /*(al >0.5 && al<1) || (ar>0.5 && ar <1)*/ ) ){
	if (y[left_startend[0]] >115 )
		steer = 1300 - left_startend[0];
	else
		steer = 1300 - right_startend[1];
	printf("steering : case 2 (%d , %d): sudden steer when straight \n",y[left_startend[0]],y[right_startend[0]]);
}

else if ((y[left_startend[1]] >115 || y[right_startend[0]] >115) && ( al < -1 || ar < -1 ) /*(al <-0.5) || (ar<-0.5))*/){
	if(y[left_startend[1]]>115 )
		steer = 1700 + (320 - left_startend[1]);

	else
		steer = 1700 + (320 - right_startend[0]);
	printf("steering : case 3 (%d , %d): sudden steer when straight \n",y[left_startend[1]],y[right_startend[1]]);
}

else if( ( y[left_startend[0]] >115 || y[right_startend[1]] > 115 ) && ( (al<1 &&al >0) || (ar>0 &&ar<1)) ){
	if (y[left_startend[0]] >115 )
		steer = 1050 - left_startend[0];
	else
		steer = 1050 - right_startend[1];
	printf("steering : case 4 (%d , %d): sudden steer when curve \n",y[left_startend[0]],y[right_startend[0]]);
}

else if ((y[left_startend[1]] >115 || y[right_startend[0]] >115) && ( (al>-1 && al <0) || (ar<0 && ar>-1))){
	if(y[left_startend[1]]>115 )
		steer = 1950 + (320 - left_startend[1]);

	else
		steer = 1950 + (320 - right_startend[0]);
	printf("steering : case 5 (%d , %d): sudden steer when curve \n",y[left_startend[1]],y[right_startend[1]]);
}

else if(y[left_startend[0]] > turn_threshold && (left_startend[1]-left_startend[0]) > 80 &&al < 0.5 && al > 0){
	steer = midsteer -(int)x1; //todayedit
	printf("steering : case 6 : turning right at curve \n");
	// printf("left_startend %d,%d",y[left_startend[0]],y[left_startend[1]]);
}
else if(y[right_startend[1]] > turn_threshold && (right_startend[1]-right_startend[0]) > 80 &&ar > -0.5 && ar < 0){

	steer = midsteer -((int)x2 -320);
	printf("steering : case 7 : turning left at curve \n");
	// printf("right_startend %d,%d",y[right_startend[0]],y[right_startend[1]]);
}

//if only one line detected
else if ( (al !=1000 && ar == 1000 )|| (al !=1000 && al > 0.5 && ar > -0.5) ){
	steer = midsteer -(int)x1;
	printf("steering : case 8 : only left line detected \n");
}

else if ( (al == 1000 && ar != 1000) || (ar < -0.5 && al < 0.5 && ar != 1000) ){
	steer = midsteer -((int)x2 -320);
	printf("steering : case 9 : only right line detected \n");
}

else {
	//steer = (*tomsteer)*2/3 +(midsteer)/3;
	steer = (*tomsteer + midsteer)/2;
	printf("steering : case null\n");
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



int yellow_line(IplImage *img, int *dashline_on){
	//detect only yellow
	int h = img->height;
	int w = img->width;
	int d = img->nChannels;
	int step = 8;
	unsigned char R,G,B;
	//int wf = 0;
	int yf = 0, bf = 0;
	int y[4] = {0,}, b[4] = {0,};
	int cd = 0, cb = 0, ch = 0;
	int i, j;
	//editwhen
	if(rotaryFlag == 2 || *dashline_on != 0)
		return 0; 

	if(hillFlag>35)
		return 0;
	
	//SAT hillflag == 1 
	if( hillFlag == 35)
	{
		DesireSpeed_Write(newspeed);
		hillFlag++;
		return 0;
	}

	if(hillFlag > 0 )
	{
		hillFlag++;
		hoho(img);
		if( hillFlag == 30){
			DesireSpeed_Write(150);
			//hillFlag++;
		}
		return 0;
	}
	

	for(i = h-1; i>= h/2; i--){
		yf = 0;
		bf = 0;
		y[0] =0, y[1] =0, y[2] = 0, y[3] = 0;
		b[0] =0, b[1] =0, b[2] = 0, b[3] = 0;
		for(j = 0; j < w; j++){
			B = (uchar)img->imageData[d*w*i + d*j + 0];

			if(yf == 0 && bf == 0 && B > 200){ //Initial yellow
				y[0]++;
			}
			else if(y[0] != 0 && bf == 0 && B < 10){ //first black
				yf = 1;
				b[0]++;
			}
			else if(b[0] != 0 && yf == 1 && B > 200){ //second y
				bf = 1;
				y[1]++;
			}
			else if(y[1] != 0 && bf == 1 && B < 10){//2nd black
				yf = 2;
				b[1]++;
			}
			else if(b[1] != 0 && yf == 2 && B > 200){ //3rd y
				bf = 2;
				y[2]++;
			}
			else if(y[2] != 0 && bf == 2 && B < 10){//3rd black
				yf = 3;
				b[2]++;
			}
			else if(b[2] != 0 && yf == 3 && B > 200){ //4th y
				bf = 3;
				y[3]++;
			}
			else if(y[3] != 0 && bf == 3 && B < 10){//4th black
				yf = 3;
				b[3]++;
			}
		}//forj

		//if(b[2] != 0 && (b[1] < 30 || b[0] < 30)) cd++;
		if(i <= 150 && y[0] == 0) ch++;
		else if(i >= 180 && y[1] != 0 ) cb++;

		if(hillFlag <= 0 && ch > 25 && cb > 20){
			hillFlag++;
			printf("hilltop\n");
			hoho(img);
			//moveSpeed(1525, 150);
			//usleep(1500000);
			//moveSpeed(1525, 180);
			return 1;
		} 
	}//fori

	return 0;	
}//0



/*
   10/16 Edit
   흰색 정지선과 점선을 구분하는 함수
   영상에서 가로를 훑으면서 
   일정 길이의 흰색, 검은색, 흰색을 만나는지에 따라서 구분..
 */
int white_line(IplImage *img, int* dashline_on, int* stopline_on){//0
	int h = img->height;
	int w = img->width;
	int d = img->nChannels;
	int step = 8;
	int whiteflag =0;
	unsigned char R,G,B;
	int countWhite1 = 0;
	int countWhite2 = 0;
	int countBlack = 0;
	int chunk = 0;
	int i, j;

	if(*stopline_on != 0)
		return 0;
	if(rotaryFlag==2){
		rotaryFlag = 3;
		return 0;
	}	
	if(rotaryFlag==3){ //in the rotary
		for(j = w/2; j < w; j++){
			for(i = h-30; i >= h/2; i--){

				B = (uchar)img->imageData[d*w*i + d*j + 0];
				if (B > 200){//3
					countWhite1 ++;
				}//3

			}//fori
		}//forj
		//printf("white : %d\n", countWhite1)
		if(countWhite1 > 1000){//5
			printf("white : %d\n", countWhite1);
			rotaryFlag=4;
		}//5
		return 0;
	}//rotaryflag


	else{
		for(i = h-1; i>= 2*h/3; i--){//1

			//printf("%d %d %d\n", countWhite1, countWhite2, countBlack);
			countWhite1 = 0, countWhite2 = 0;
			countBlack = 0;
			for(j = 0; j < w; j++){//2
				//RGB same = 255 or 0; so check one
				B = (uchar)img->imageData[d*w*i + d*j + 0];
				if (B > 200){//3
					if(countBlack == 0){//4
						countWhite1 ++;
						if(countWhite1 > 250){//5
							printf("result : stopline\n");
							*stopline_on=1;

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
		}
	}

	return 0;
}//0 


/*
   FOR RECORD
 */
void K_PARKH(void){
int steer = 1525;
	int s2;				// DistanceSensor(2)


	printf(" ^^^^^^^^^^^ PARK H IN ^^^^^^^^^^^^^^^^\n\n");

	moveSpeed(steer ,0);

	s2 = DistanceSensor(2);   

	if (s2 < 700) {			// outside
		printf("$$$$$$$$$$$$$ case 1 : outside \n");
		moveSpeed(1120, -200);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1150, 200);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2500 && DistanceSensor(3) <= 3500){}
	}

	else if (s2 < 1000)	{	// middle
		printf("$$$$$$$$$$$$$ case 2 : middle \n");
		moveSpeed(1100, -200);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1100, 200);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2300 && DistanceSensor(3) <= 3200){}

	}

	else {					// inside
		printf("$$$$$$$$$$$$$ case 3 : inside \n");
		moveSpeed(1100, -200);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1100, 200);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2500 && DistanceSensor(3) <= 3500){}
	}
	
	printf("s2 : %d, s3 : %d\n", DistanceSensor(3), DistanceSensor(2));
	if( DistanceSensor(3) < 1600 && (DistanceSensor(3) - DistanceSensor(2)) > 500)
	{
		ggoom = 1;
		
		moveSpeed(1200, 100);
		while(1)
		{
			if(DistanceSensor(1) > 2500 || DistanceSensor(2) > 2500)
				break;
		}
		
	}
	

	moveSpeed(steer,0);

	Alarm_Write(ON);
	usleep(500000);
	Alarm_Write(OFF);

	if(ggoom)
	{
		ggoom = 0;
		moveSpeed(1200, -100);
		while(1)
		{
			if(DistanceSensor(4) > 3000 || DistanceSensor(3) > 3500)
				break;
		}
	}

	moveSpeed(1990,120);
	while(1){
		// 3000, 2700, 600
		if(DistanceSensor(2)>=2000 || DistanceSensor(1)>=2500){
			printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			break;
		}
	}

	// -90 -> -120
	moveSpeed(1200,-90);
	while(1){
		if( DistanceSensor(5)>=2000 || DistanceSensor(4)>=2000 || DistanceSensor(3)>=3500)
		{
			printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
			break;
		}
	}
	//90 -> 120
	moveSpeed(1900,120);
	while(DistanceSensor(2)>=550){
	}
	moveSpeed(1100,newspeed);
	usleep(100000);

	return 0;
	//H1
}
/*
   FOR RECORD
 */
void K_PARKV(void){
	/*
	//usleep(50000);
	moveSpeed(1000,-180);
	usleep(1300000);
	//usleep(50000);
	//후방센서 감지시 멈추기
	moveSpeed(1525,-120);
	while(DistanceSensor(4)<2000 || DistanceSensor(5) < 3500){}
	moveSpeed(1525,0);

	Alarm_Write(ON);
	sleep(1);
	Alarm_Write(OFF);

	moveSpeed(1525,150);
	usleep(700000);
	moveSpeed(1000,newspeed);
	usleep(600000);
*/
//usleep(50000);
    usleep(20000);
	moveSpeed(1020,-180);
	while(1){
		if( DistanceSensor(5) >= 3200){
			moveSpeed(1700, 130);
			while(1){
				if( DistanceSensor(3) <= 600 & DistanceSensor(5) <=600)
					break;
			}
			break;
		}
		else if ( DistanceSensor(3) >= 1500 && DistanceSensor(5) >= 1500)
	break;

	}
	moveSpeed(1525,-150);
	while(1){
		if(DistanceSensor(4) >= 1800 || (DistanceSensor(2)>=1500 && DistanceSensor(6)>=1500) )
		break;
	}
	moveSpeed(1525,0);

	Alarm_Write(ON);
	sleep(1);
	Alarm_Write(OFF);

	moveSpeed(1525,150);
	usleep(400000);
	moveSpeed(1000,newspeed);
	usleep(600000);

}

//HORIZONTAL PARKING
/*
   FOR MISSION
 */
void parkH(void){
	int steer = 1525;
	int s2;				// DistanceSensor(2)


	printf(" ^^^^^^^^^^^ PARK H IN ^^^^^^^^^^^^^^^^\n\n");

	moveSpeed( steer, DesireSpeed_Read());
	usleep(200000);

	moveSpeed(steer ,0);

	s2 = DistanceSensor(2);   

	if (s2 < 700) {			// outside
		printf("$$$$$$$$$$$$$ case 1 : outside \n");
		moveSpeed(1120, -150);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1150, 150);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2500 && DistanceSensor(3) <= 3500){}
	}

	else if (s2 < 1000)	{	// middle
		printf("$$$$$$$$$$$$$ case 2 : middle \n");
		moveSpeed(1100, -150);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1100, 150);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2300 && DistanceSensor(3) <= 3200){}

	}

	else {					// inside
		printf("$$$$$$$$$$$$$ case 3 : inside \n");
		moveSpeed(1100, -150);
		while (1) {
			if (DistanceSensor(4) >= 1200 /*&& DistanceSensor(3) >= 2000*/)
				break;
		}
		moveSpeed(1100, 150);
		while(1) {
			if(DistanceSensor(4) < 540)
				break;
		}
		moveSpeed(1990, 0);
		usleep(100000);
		moveSpeed(1990, -130);
		while(DistanceSensor(4)<=2500 && DistanceSensor(3) <= 3500){}
	}
	
	printf("s2 : %d, s3 : %d\n", DistanceSensor(3), DistanceSensor(2));
	if( DistanceSensor(3) < 1600 && (DistanceSensor(3) - DistanceSensor(2)) > 500)
	{
		ggoom = 1;
		
		moveSpeed(1200, 100);
		while(1)
		{
			if(DistanceSensor(1) > 2500 || DistanceSensor(2) > 2500)
				break;
		}
		
	}
	

	moveSpeed(steer,0);

	Alarm_Write(ON);
	usleep(500000);
	Alarm_Write(OFF);

	if(ggoom)
	{
		ggoom = 0;
		moveSpeed(1200, -100);
		while(1)
		{
			if(DistanceSensor(4) > 3000 || DistanceSensor(3) > 3500)
				break;
		}
	}

	moveSpeed(1990,120);
	while(1){
		// 3000, 2700, 600
		if(DistanceSensor(2)>=2000 || DistanceSensor(1)>=2500){
			printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
			break;
		}
	}

	// -90 -> -120
	moveSpeed(1200,-90);
	while(1){
		if( DistanceSensor(5)>=2000 || DistanceSensor(4)>=2000 || DistanceSensor(3)>=3500)
		{
			printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
			break;
		}
	}
	//90 -> 120
	moveSpeed(1900,120);
	while(DistanceSensor(2)>=550){
	}
	moveSpeed(1100,newspeed);
	usleep(100000);

	return 0;
	//H1
}

void SensorCheck(){
	printf(" Front : %d     right_front : %d    right_rear : %d    Behind : %d     left_rear : %d\n\n", DistanceSensor(1),DistanceSensor(2),DistanceSensor(3), DistanceSensor(4),DistanceSensor(5));
	usleep(500000);
}


void hoho(IplImage *img){

	int mid = (lowerBound + upperBound)/2;
	int h = img->height;
	int w = img->width;
	int d = img->nChannels;
	int i,j=0;
	unsigned char R,G,B;
	int gp1;
	int hohoAfter =0;
	int ii,jj,avg;

	printf("lB %d , uB : %d\n", lowerBound, upperBound);

	while(1){
		for (i=159;i>=0 ; i--){
			B = (uchar)img->imageData[d*w*mid + d*i + 0];
			G = (uchar)img->imageData[d*w*mid + d*i + 1];
			R = (uchar)img->imageData[d*w*mid + d*i + 2];
			if(B==255 && G==255 && R==255)
				break;	
		}
		if(i<0){
			mid += 1;
			if(mid>=lowerBound)
				break;
			continue;
		}

		for (j=160;j<320 ; j++){
			B = (uchar)img->imageData[d*w*mid + d*j + 0];
			G = (uchar)img->imageData[d*w*mid + d*j + 1];
			R = (uchar)img->imageData[d*w*mid + d*j + 2];
			if(B==255 && G==255 && R==255)
				break;	
		}
		if(j>=320){
			mid += 1;
			if(mid>=lowerBound)
				break;
			continue;
		}

		break;
	}

	if(i==-1 || j==320)
	{
		PositionControlOnOff_Write(UNCONTROL);
		SpeedControlOnOff_Write(CONTROL);
		DesireSpeed_Write(150);

		printf("NOBADAK %d\n", SteeringServoControl_Read());
		hillFlag=30;
		stopline_on = 0;

		return;
	}

	printf("hihihihihihihihihi %d\n", (i+j)/2);
	if( (i+j)/2 < 160){
		moveSpeed(1580,hillSpeed);

	}
	else{
		moveSpeed(1470,hillSpeed);
	}

	if(hillSpeed >= 112)
		hillSpeed -= 2;
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
	int final[400];

	//DRIVE PAR
	int midsteer = 1530;
	int tomsteer=midsteer;
	

	//REDDETECT PAR
	int red_stop =0;
	int red_on = 0;
	int cur_reddetect = 0;
	int pre_reddetect = 0;


	int bump_on=1;
	int curr_dash = 0;
	int prev_dash = 0;
	int sflag = 0;
	int dashline_on = 0;


	//TFL PARAMETER
	int tfl_direction = 0;

	int max_hill_threshold = 0;

	int cnt_final = 0;


	// 11.4 OVERTAKING PAR
	int B, G, R;
	int w;
	int d;
	int ii, jj;
	int overTakingStop;
	int afterOver =0;


	// PARKING PAR
	int ppre, preDis , curDis ;
	int dflag = 0;
	int fcount = 0, fcount2 =0;
	int parkdetect = 10;
	int typeCheck = 1;

	// INIT
	CarControlInit();

	sleep(1);

	CameraXServoControl_Write(1480);
	CameraYServoControl_Write(1700);
	SteeringServoControl_Write(midsteer);

	sleep(1);

	PositionControlOnOff_Write(UNCONTROL);
	SpeedControlOnOff_Write(CONTROL);
	DesireSpeed_Write(newspeed);

	ppre = 0, preDis = 0;
	curDis = DistanceSensor(3);



	//------------------------------------------------------------

	IplImage* imgOrigin;
	// cvCreateImage
	imgOrigin = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);


	while(1){

		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond, &mutex);


		GetTime(&pt1);
		ptime1 = (NvU64)pt1.tv_sec * 1000000000LL + (NvU64)pt1.tv_nsec;


		Frame2Ipl(imgOrigin); // save image to IplImage structure & resize image from 720x480 to 320x240
		pthread_mutex_unlock(&mutex);


		init_array(steer_y, 400, 1000);
		init_array(hill_y, 400, 1000);
		init_array(final, 400, 1000);

		//TODO
		// imgOrigin -- > binary_image
		IplImage **bimgs = binary_image_array(imgOrigin, steer_y, hill_y, final,&cnt_final, &dashline_on, &stopline_on);
/*
		sprintf(fileName, "captureImage/%d.png", i);
		cvSaveImage(fileName,imgOrigin ,0);

		sprintf(fileName, "captureImage/0_%d.png", i);
		cvSaveImage(fileName,bimgs[0] ,0);

		if(rotaryFlag > 0 || dashline_on == 1){      
			sprintf(fileName, "captureImage/1_%d.png", i);
			cvSaveImage(fileName,bimgs[1] ,0);
		}

		sprintf(fileName, "captureImage/2_%d.png", i);
		cvSaveImage(fileName,bimgs[2] ,0);
*/

		// 11.4  FOR OVERTAKING
		//차로추월 영상으로 복귀하다가 마지막만 usleep 
		if( overTakingFlag==1)
		{
			// dashline - > 1, 0 , -1



			//원래는 ii에 차선변경을 몇차선으로 했는지에 따라 다른 값이 들어가지만
			//현재는 무조건 1차선으로 변경하기때문에 10이란 값으로 고정

			ii = 10;
			jj = 80;

			w = bimgs[0]->width;
			d = bimgs[0]->nChannels;
			while(jj++<239) 
			{
				B = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 0];
				G = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 1];
				R = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 2];
				if(B==255 && G==255 && R==255)
					break;	

			}	
			i++;

			if(jj>239) {
				overTakingFlag=2;
				moveSpeed(1525, 0);
				usleep(100000);

				ii = ii == 10 ? 0 : 160;
				moveSpeed(1525, 140);

			}
			continue;
		}

		else if( overTakingFlag==2){

			jj = 140;
			overTakingStop = ii + 159;

			w = bimgs[0]->width;
			d = bimgs[0]->nChannels;
			while(ii++<overTakingStop) {
				B = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 0];
				G = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 1];
				R = (uchar)bimgs[0]->imageData[d*w*jj + d*ii + 2];
				if(B==255 && G==255 && R==255){
					overTakingFlag=3;


					moveSpeed(1900, 150);
					usleep(1300000);
					break;
				}
			}
			if(ii!=160){
				printf("ii:%d\n",ii);

			}
			i++;
			ii=0;

			continue;
		}

		else if( overTakingFlag==3){
			//printf(" 44444444\n");
			moveSpeed(1540,0);
			//usleep(5000000);
			overTakingFlag = -1;
			
			Alarm_Write(ON);
			sleep(1);
			Alarm_Write(OFF);
			tfl_on = 1;
		}


		int num_y = 320;

		float left_tangent, right_tangent;
		int left_startend[2] = {0,0};
		int right_startend[2] = {0,0};


		checkline(steer_y,num_y,left_startend,right_startend,&left_tangent,&right_tangent);


		if (hillFlag < 1 || hillFlag >= 30)
		{
			checkdir(steer_y,left_startend,right_startend,&left_tangent,&right_tangent,&tomsteer,midsteer, bimgs[0]);
			SteeringServoControl_Write(tomsteer);
		}

		white_line(bimgs[2], &dashline_on, &stopline_on);
		yellow_line(bimgs[0], &dashline_on);

		

		if(rotaryFlag== 3 &&  DistanceSensor(4) > 600){
			rotarySpeed += 20;
			rotarySpeed = rotarySpeed <= 160? rotarySpeed : 160;  			
			DesireSpeed_Write(rotarySpeed);
		}

		if(dashline_on==0){

			//PARKDETCET
			ppre = preDis;
			preDis = curDis;
			curDis = DistanceSensor(3);

			if(parkVflag == 1 && curDis < 100){
				printf("parkV reset\n");
				parkVflag = 2;
				dflag = 0;
				typeCheck = 1;
			}
			else if(parkHflag == 1 && curDis < 100){
				printf("ParkH reset\n");
				parkHflag = 2;
				dflag = 0;
				typeCheck = 1;
			}
			if(parkVflag == 0 || parkHflag == 0){

				if ( dflag == 0 ){
					//400 -> 300
					if(curDis - ppre >= 300){
						printf("#####first rising#####\n");
						dflag = 1;
						fcount = i;
						//Timer start
					}
				}

				else if ( dflag == 1){
					//have to fall in 5 frame
					if( i - fcount > 13) {
						dflag = 0; 
						printf("$$$$$reset$$$$$\n");
					} //timer reset
					else{
						//printf("flag : %d\n", i-fcount);
						// 450 --> 300
						if(ppre - curDis >= 300) {
							printf("flag : %d\n", i-fcount);
							dflag = 2;
							printf("#####first falling######\n");
							//Timer Start
							fcount2 = i;
						}
					}
				}

				else if ( dflag == 2){
					if(typeCheck)
					{
						//printf("s2 : %d , s3 : %d\n", DistanceSensor(2) , DistanceSensor(3));
						if(abs(DistanceSensor(2) - DistanceSensor(3)) > 200 || parkHflag) {
							parkdetect = 3;
						}
						typeCheck = 0;
					}

					if(i - fcount2 > 20) {
						dflag = 0; 
						printf("$$$ohhae$$$$\n");
						typeCheck = 1;
					}

					else{
						// 450 --> 300
						if(curDis - ppre >= 300){
							//--> parking start
							printf("flag2 : %d\n", i-fcount2);
							//parkdetect = i - fcount2;

							//printf("parkdetect : %d\n", parkdetect);
							if(parkHflag == 0 && parkdetect >= 7){
								//180
								printf("parkH STOP~P~P~PPP~P\n");
								parkHflag = 1;
								K_PARKH();
								//parkH();
							}

							else if(parkVflag == 0 && parkdetect < 5){
								printf("parkV STOP~P~P~P~P~P~P~P~P\n");
								parkVflag = 1;
								K_PARKV();
							}			

							printf("##################parking   end\n");
							dflag = 0;
							typeCheck = 1;
						}
					}
				}
			}

			//REDDETECT SOURCE
			if(red_on == 0){ //original - 0
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
					DesireSpeed_Write(newspeed); //original : 140
					pre_reddetect = 0;
					red_on = 1;
					//bump_on = 2;
				}
			}

			//ROTARY STOPLINE DETECT
			if(stopline_on==1){

				printf("stop and watch\n");
				moveSpeed(1510, 0);
				rotaryFlag=1;
			}


			////////OUT OF ROTARY
			if(rotaryFlag==4){//out of rotary
				moveSpeed(1700, newspeed);
				usleep(700000);
				stopline_on = 100;
				rotaryFlag=-1; //end of rotary
			}

		}//dashline

		/*if(dashline_on==1 && stopline_on==1){		
			//moveSpeed(1700, 120);			
			//usleep(300000);
			//overTakingFlag = 4;		
			moveSpeed(1525, 0);
			dashline_on=-1;
			tfl_on = 1;
		}*/
		
		 if(tfl_on == 1){
		   tfl(imgOrigin, &tfl_on, &tfl_direction, &max_hill_threshold);
		//printf("cnt_final [%d]",cnt_final);
		}
		if(tfl_on == 2){
			printf("tfl_center : %d\n", tflCenter);
			moveSpeed(1540 + (160 - tflCenter), 150);
			tfl_on = 3;
		}

		if(tfl_on == 4){
			if(tfl_direction == 1)
			{
				if(tflCenter>=200)
					SteeringServoControl_Write(1980);
				else if(tflCenter>=150)
					SteeringServoControl_Write(1950);
				else
					SteeringServoControl_Write(1900);

				usleep(3000000);
				moveSpeed(1525, 120);
				usleep(1100000);
				moveSpeed(1525, 0);
			}
			else
			{
				SteeringServoControl_Write(1150);	
				usleep(2500000);
				moveSpeed(1525, 120);
				usleep(1100000);
				moveSpeed(1525, 0);
			}
			return 0;
		}
		/*if(checkbump2(bump2,num_y,&tomsteer,midsteer,bump_on)==1 ){
		DesireSpeed_Write(120);
		SteeringServoControl_Write(midsteer);
		tfl_on =4;
		}
		}

		if(cnt_final > 200 &&tfl_on ==4){
		DesireSpeed_Write(0);
		SteeringServoControl_Write(midsteer);
		break;*/
		
		 
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
	//printf (" FRAME ---->  %d \n\n ", frame); 
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

