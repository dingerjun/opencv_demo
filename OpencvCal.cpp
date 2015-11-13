/*******************************************************************************
*此程序用来自制一个标定用的测试方格
*******************************************************************************/
#include <highgui.h>
#include <stdio.h>

int blockSide; //方格的边长
int count = 1; //图像个数

int calculate(void);

int main()
{
while (1)
{
calculate();
/*
if (calculate())
{
break;
}
*/
}
system("pause");

return 0;
}

int calculate(void)
{
int m, n;
int i, j;
int flag = 0;
char ch = '\0';
char str[11] = {'t', 'e', 'x', 't'}; //图像名称
IplImage *img = NULL;
unsigned char *data = NULL; //指向图像矩阵首地址的指针
unsigned char *temp = NULL;
unsigned char *dstPoint = NULL;//指向要操作的元素

m = n = j = i = 0;

printf("请输入方格的边长：\n");
scanf("%d", &blockSide);
printf("请输入标定块的行和列：\n");
scanf("%d%d", &m, &n);

img = cvCreateImage(
cvSize(blockSide*n, blockSide*m),
IPL_DEPTH_8U,
1
);
data = (unsigned char *)img->imageData;

for (i = 0; i < n; i++)
{
for ( j = 0; j < m; j++)
{

flag = i + j;
/*
if (i == m-1 || j == n-1)
{
flag*/
temp = data + img->widthStep * blockSide * j + blockSide * i;
if ( flag % 2)
{
for (int k = 0; k < blockSide; k++)
{
for ( int k1 = 0; k1 < blockSide; k1++)
{
dstPoint = temp + k1 * img->widthStep + k;
*dstPoint = 255;
}
}
}
else
{
for (int k = 0; k < blockSide; k++)
{
for ( int k1 = 0; k1 < blockSide; k1++)
{
dstPoint = temp + k1 * img->widthStep + k;
*dstPoint = 0;
}
}
}
dstPoint = NULL; //重新置零
}
}

cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);//创建一个窗口
cvShowImage("Image", img);//显示图片
cvWaitKey(0);

getchar();

printf("是否保存图像？<Y/N>(按q退出):");
scanf("%c", &ch);
if (ch == 'q' || ch == 'Q')
{
exit(0);
}
else if (ch == 'Y' || ch == 'y')
{
str[4] = count / 10 + '0';
str[5] = count++ % 10 + '0';

cvSaveImage(strcat(str, ".jpg"), img);
printf("图片保存为当前工作目录下的%s\n", str);
return 1;
}
else
{
return 0;
}
//cvDestroyWindow("Image"); //销毁窗口
}
