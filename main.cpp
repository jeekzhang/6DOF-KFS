/* 6轴机器人运动正解
 * 关节角度在文件我放在"D:\\j.txt"中
 * 机器人参数在文件"D:\\dh.txt"中
 * x,y,z,rx,ry,rz在屏幕输出
 * 原代码来自https://blog.csdn.net/weixin_37942267/article/details/78806448?spm=1001.2014.3001.5502*/

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <math.h>

using namespace std;

#define XYZ_F_J "D:\\j.txt"
#define DESIGN_DT "D:\\dh.txt"

#define RAD2ANG (3.1415926535898 / 180.0)
#define ANG2RAD (180.0 / 3.1415926535898)
#define IS_ZERO(var)     \
    if (abs(var) < 1e-6) \
    {                    \
        var = 0;         \
    }

#define MATRIX_N 4

typedef struct
{
    double joint_v; // joint variable
    double length;
    double d;
    double angle;
} param_t;

double matrix_A1[MATRIX_N][MATRIX_N];
double matrix_A2[MATRIX_N][MATRIX_N];
double matrix_A3[MATRIX_N][MATRIX_N];
double matrix_A4[MATRIX_N][MATRIX_N];
double matrix_A5[MATRIX_N][MATRIX_N];
double matrix_A6[MATRIX_N][MATRIX_N];

double matrix_toolxyz[MATRIX_N][MATRIX_N];

void initmatrix_A(param_t *p_table);
void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], param_t *p_param);

void matrix_mul(double matrix_a[MATRIX_N][MATRIX_N],
                double matrix_b[MATRIX_N][MATRIX_N],
                double matrix_result[MATRIX_N][MATRIX_N]);

void matrix_add(double matrix_a[MATRIX_N][MATRIX_N],
                double matrix_b[MATRIX_N][MATRIX_N],
                double matrix_sum[MATRIX_N][MATRIX_N], int m, int n);

void matrix_copy(double matrix_a[MATRIX_N][MATRIX_N],
                 double matrix_b[MATRIX_N][MATRIX_N], int m, int n);

void initmatrix_tool(double toolx, double tooly, double toolz);

void printmatrix(double matrix[MATRIX_N][MATRIX_N], int m, int n)
{
    int i, j;

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            printf(" %lf ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void printmatrix_1(double matrix[MATRIX_N][1], int m, int n)
{
    int i, j;

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            printf(" %lf ", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

int main()
{

    double matrix_T1[MATRIX_N][MATRIX_N];
    double matrix_T2[MATRIX_N][MATRIX_N];
    double matrix_T3[MATRIX_N][MATRIX_N];
    double matrix_T4[MATRIX_N][MATRIX_N];
    double matrix_T5[MATRIX_N][MATRIX_N];
    double matrix_T6[MATRIX_N][MATRIX_N];

    double toolx = 0, tooly = 0, toolz = 0, toolrx = 0, toolry = 0, toolrz = 0;
    double worldx = 0, worldy = 0, worldz = 0, worldrx = 0, worldry = 0, worldrz = 0;

    param_t param_table[6] = {0};
    memset(param_table, 0, sizeof(param_table));

    FILE *fp = NULL;

    fp = fopen(XYZ_F_J, "r");
    if (fp == NULL)
    {
        perror("open J1_J6 file error\n");
        return 0;
    }
    fscanf(fp, "%lf%lf%lf%lf%lf%lf",
           &param_table[0].joint_v,
           &param_table[1].joint_v,
           &param_table[2].joint_v,
           &param_table[3].joint_v,
           &param_table[4].joint_v,
           &param_table[5].joint_v);
    printf("j1...j6\n%lf %lf %lf %lf %lf %lf\n",
           param_table[0].joint_v,
           param_table[1].joint_v,
           param_table[2].joint_v,
           param_table[3].joint_v,
           param_table[4].joint_v,
           param_table[5].joint_v);

    // 将机器人关节角度转换成弧度
    param_table[0].joint_v *= RAD2ANG;
    param_table[1].joint_v *= RAD2ANG;
    param_table[2].joint_v *= RAD2ANG;
    param_table[3].joint_v *= RAD2ANG;
    param_table[4].joint_v *= RAD2ANG;
    param_table[5].joint_v *= RAD2ANG;

    fclose(fp);

    fp = fopen(DESIGN_DT, "r");
    if (fp == NULL)
    {
        perror("open param_table file error\n");
        return 0;
    }

    // 读入关节参数
    int i;
    for (i = 0; i < 6; i++)
    {
        fscanf(fp, "%lf%lf%lf",
               &param_table[i].length,
               &param_table[i].d,
               &param_table[i].angle);
    }
    fclose(fp);

    param_table[0].angle *= RAD2ANG;
    param_table[1].angle *= RAD2ANG;
    param_table[2].angle *= RAD2ANG;
    param_table[3].angle *= RAD2ANG;
    param_table[4].angle *= RAD2ANG;
    param_table[5].angle *= RAD2ANG;

    initmatrix_A(param_table);

    计算变换矩阵 matrix T1-- - T6
                                   matrix_copy(matrix_A1, matrix_T1, MATRIX_N, MATRIX_N);

    matrix_mul(matrix_T1, matrix_A2, matrix_T2);

    matrix_mul(matrix_T2, matrix_A3, matrix_T3);

    matrix_mul(matrix_T3, matrix_A4, matrix_T4);

    matrix_mul(matrix_T4, matrix_A5, matrix_T5);

    matrix_mul(matrix_T5, matrix_A6, matrix_T6);

    float sy = sqrt(matrix_T6[0][0] * matrix_T6[0][0] + matrix_T6[0][1] * matrix_T6[0][1]);
    bool singular = sy < 1e-6; // If

    float rx, ry, rz;
    if (!singular)
    {
        rx = atan2(-matrix_T6[1][2], matrix_T6[2][2]) * ANG2RAD;
        ry = atan2(matrix_T6[0][2], sy) * ANG2RAD;
        rz = atan2(-matrix_T6[0][1], matrix_T6[0][0]) * ANG2RAD;
    }
    else
    {
        rx = atan2(matrix_T6[2][1], matrix_T6[1][1]) * ANG2RAD;
        ry = atan2(matrix_T6[0][2], sy) * ANG2RAD;
        rz = 0;
    }
    IS_ZERO(rx);
    IS_ZERO(ry);
    IS_ZERO(rz);

    printf("\n----curent x, y, z, rx, ry,rz-----\n%lf \n %lf\n %lf\n %lf \n %lf\n %lf\n ",
           matrix_T6[0][3], matrix_T6[1][3],
           matrix_T6[2][3], rx, ry, rz);
}

void initmatrix_A(param_t *p_table)
{ // 计算关节坐标矩阵 matrix A1--A6
    calculate_matrix_A(matrix_A1, p_table + 0);

    calculate_matrix_A(matrix_A2, p_table + 1);

    calculate_matrix_A(matrix_A3, p_table + 2);

    calculate_matrix_A(matrix_A4, p_table + 3);

    calculate_matrix_A(matrix_A5, p_table + 4);

    calculate_matrix_A(matrix_A6, p_table + 5);
}

void calculate_matrix_A(double matrix[MATRIX_N][MATRIX_N], param_t *p_param)
{ // 根据关节参数计算矩阵
    double *pmatrix = (double *)matrix;
    double value, var_c, var_s, angle_c, angle_s;

    var_c = cos(p_param->joint_v);
    IS_ZERO(var_c);
    var_s = sin(p_param->joint_v);
    IS_ZERO(var_s);
    angle_c = cos(p_param->angle);
    IS_ZERO(angle_c);
    angle_s = sin(p_param->angle);
    IS_ZERO(angle_s);

    *pmatrix++ = var_c;

    *pmatrix++ = -var_s * angle_c;

    *pmatrix++ = var_s * angle_s;

    *pmatrix++ = p_param->length * var_c;

    *pmatrix++ = var_s;

    *pmatrix++ = var_c * angle_c;

    *pmatrix++ = -var_c * angle_s;

    *pmatrix++ = p_param->length * var_s;

    *pmatrix++ = 0;
    *pmatrix++ = angle_s;
    *pmatrix++ = angle_c;
    *pmatrix++ = p_param->d;

    *pmatrix++ = 0;
    *pmatrix++ = 0;
    *pmatrix++ = 0;
    *pmatrix = 1;
}

void initmatrix_tool(double toolx, double tooly, double toolz)
{
    matrix_toolxyz[0][0] = toolx;
    matrix_toolxyz[1][0] = tooly;
    matrix_toolxyz[2][0] = toolz;

    /* 初始化 toolrx, tooly, toolz */
}

void matrix_mul(double matrix_a[MATRIX_N][MATRIX_N],
                double matrix_b[MATRIX_N][MATRIX_N],
                double matrix_result[MATRIX_N][MATRIX_N])
{
    int i, j, k;
    double sum;

    /*嵌套循环计算结果矩阵（m*p）的每个元素*/
    for (i = 0; i < MATRIX_N; i++)
        for (j = 0; j < MATRIX_N; j++)
        {
            /*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
            sum = 0;
            for (k = 0; k < MATRIX_N; k++)
                sum += matrix_a[i][k] * matrix_b[k][j];
            matrix_result[i][j] = sum;
        }
}

void matrix_add(double matrix_a[MATRIX_N][MATRIX_N],
                double matrix_b[MATRIX_N][MATRIX_N],
                double matrix_sum[MATRIX_N][MATRIX_N], int m, int n)
{
    int i, j;

    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            matrix_sum[i][j] = matrix_a[i][j] + matrix_b[i][j];
        }
    }
}

void matrix_copy(double matrix_src[MATRIX_N][MATRIX_N],
                 double matrix_des[MATRIX_N][MATRIX_N], int m, int n)
{
    int i, j;
    for (i = 0; i < m; i++)
    {
        for (j = 0; j < n; j++)
        {
            matrix_des[i][j] = matrix_src[i][j];
        }
    }
}
