/*******************************************************************************
24 Color checker
*******************************************************************************/
#include <highgui.h>
#include <stdio.h>

int mLine = 4;
int mColumn = 6;
int mBlankSize = 5; 
int mBlockSize = 30;
int mHeight;
int mWidth;
IplImage *mImage;

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} Rect;

typedef struct {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} Color;

Color mColor[] = {
     {115,82,68},
     {194,150,130},
     {98,122,157},
     {87,108,67},
     {133,128,177},
     {103,189,170},
     {214,126,44},
     {80,91,166},
     {193,90,99},
     {94,60,108},
     {157,188,64},
     {224,163,46},
     {56,61,150},
     {70,148,73},
     {175,54,60},
     {231,199,31},
     {187,86,149},
     {8,133,161},
     {243,243,242},
     {200,200,200},
     {160,160,160},
     {122,122,121},
     {85,85,85},
     {52,52,52}
};

void initParameters(){
    mLine = 4;
    mColumn = 6;
    mBlankSize = 50; 
    mBlockSize = 200;
    mHeight = mBlockSize*mLine + mBlankSize*(mLine +1);
    mWidth = mBlockSize*mColumn + mBlankSize*(mColumn +1);
    printf("mHeight  = %d\n", mHeight);
    printf("mWidth = %d\n", mWidth);
    mImage = cvCreateImage( cvSize(mWidth, mHeight), IPL_DEPTH_8U, 3);
}

int getRect(int index, Rect *rect){
    if ( index < 0 || index >= mLine*mColumn)
        return -1;
    int x = index%mColumn;
    int y = index/mColumn;
    rect->left = mBlankSize +(mBlankSize + mBlockSize) *x ;
    rect->top = mBlankSize +(mBlankSize + mBlockSize) *y ;
    rect->right = rect->left + mBlockSize;
    rect->bottom = rect->top + mBlockSize;
    printf("left  = %d\n", rect->left);
    printf("top = %d\n", rect->top);
    printf("right  = %d\n", rect->right);
    printf("bottom = %d\n", rect->bottom);
    return 0;
}

int drawBlock(int index , Color color){
     Rect block = {0,0,0,0}; 
     int ret = getRect(index , &block);
     if (ret < 0)
         return -1;
     int i ,j;
     unsigned char * head = NULL; 
     head = (unsigned char *)mImage->imageData;
     int linelength = mImage->widthStep;
     printf("linelength = %d\n", linelength);
     printf("linelength1 = %d\n", mImage->widthStep);
     for(j=block.top;j <block.bottom;j++){
         unsigned char *data = head + (linelength * j )+ block.left*3;
         unsigned char *end = head + (linelength * j )+ block.right*3; 
         while (data < end) {
             *data++ = color.blue;
             *data++ = color.green;
             *data++ = color.red;
         }
     }
}

int draw24ColorPicture() {
    unsigned char * head = NULL; 
    head = (unsigned char *)mImage->imageData;
    memset(head, 0x0 , mWidth*mHeight*3);
    int indexEnd = sizeof(mColor)/sizeof(Color);
    printf("index end = %d", indexEnd);
    int i;
    for (i =0;i< indexEnd; i++){
        drawBlock(i, mColor[i]);
    }
}

int show(void)
{
char ch = '\0';
char str[11] = {'t', 'e', 'x', 't'}; 
cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
cvShowImage("Image", mImage);
cvWaitKey(0);
return 1;
}

int main()
{
    initParameters();
    draw24ColorPicture();
    show();

    return 0;
}


