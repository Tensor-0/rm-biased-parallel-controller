
#include "Control_Task.h"
#include "INS_Task.h"//陀螺仪
#include "cmsis_os.h"//FreeRTOS
#include "bsp_uart.h"//串口
#include "bsp_adc.h"//ADC--电压检测--低电压警告
#include "Remote_Control.h"//遥控器
#include "Image_Transmission.h"
#include "Ramp.h"//斜坡函数和移动平均滤波器，用于控制系统的信号平滑处理
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"


//float K11[6] = {0,-67.224661f,134.050893f,-176.675264f,-11.203565f};
//float K12[6] = {0,4.666486f,-12.002738f,-32.983282f,0.087257f};
//float K13[6] = {0,-4.955885f,7.400313f,-3.806159f,-10.789851f};
//float K14[6] = {0,-5.494303f,10.260238f,-11.461555f,-14.246477f};
//float K15[6] = {0,-171.118016f,270.900410f,-155.298000f,47.386522f};
//float K16[6] = {0,-9.626790f,15.857921f,-9.589570f,4.186371f};
//float K21[6] = {0,-31.521987f,47.548063f,-14.779719f,17.176591f};
//float K22[6] = {0,-3.326380f,6.122143f,2.110718f,1.696890f};
//float K23[6] = {0,-35.678434f,55.842882f,-31.088140f,8.445611f};
//float K24[6] = {0,-46.453674f,72.316917f,-39.387661f,11.171125f};
//float K25[6] = {0,98.580421f,-148.868128f,78.405122f,128.964556f};
//float K26[6] = {0,10.240428f,-16.008535f,8.937617f,3.723793f};




// float K11[6] = {0,-160.192544f,220.644461f,-202.842431f,-8.648239f};
// float K12[6] = {0,13.906644f,-20.556364f,-30.413768f,-0.162286f};
// float K13[6] = {0,-34.506193f,34.296896f,-11.750686f,-10.031053f};
// float K14[6] = {0,-26.072423f,29.036147f,-17.021058f,-13.714199f};
// float K15[6] = {0,-791.841590f,840.770292f,-325.097137f,63.746935f};
// float K16[6] = {0,-33.806130f,38.224682f,-16.304005f,4.838057f};
// float K21[6] = {0,-154.616893f,160.600039f,-48.478949f,20.425160f};
// float K22[6] = {0,-13.974165f,15.923287f,-0.817485f,1.979801f};
// float K23[6] = {0,-168.442887f,177.729400f,-67.405642f,11.944916f};
// float K24[6] = {0,-230.199988f,240.766773f,-89.506822f,15.993289f};
// float K25[6] = {0,654.411995f,-655.046460f,227.994254f,114.669705f};
// float K26[6] = {0,51.379870f,-53.701862f,20.146102f,2.646013f};


/*     Q=diag([700 1 100 100 2000 1 ]); 
//     R=diag([0.75  0.25]);  */
//float K11[6] = {0,-203.960729f,244.137404f,-193.257587f,-21.020228f};
//float K12[6] = {0,8.652812f,-14.927657f,-32.621243f,-0.287868f};
//float K13[6] = {0,-24.683842f,25.840376f,-9.506651f,-10.203650f};
//float K14[6] = {0,24.022407f,-22.733357f,2.459290f,-17.075029f};
//float K15[6] = {0,-538.651979f,598.832247f,-247.879687f,52.120226f};
//float K16[6] = {0,-32.326421f,37.002538f,-16.064382f,4.816248f};
//float K21[6] = {0,-443.998414f,493.299321f,-194.377784f,48.161445f};
//float K22[6] = {0,1.982760f,0.548605f,2.815860f,2.058602f};
//float K23[6] = {0,-101.442330f,117.778795f,-50.959002f,10.648799f};
//float K24[6] = {0,-178.962300f,199.929639f,-81.730507f,16.198186f};
//float K25[6] = {0,637.691625f,-652.497507f,233.905673f,61.221087f};
//float K26[6] = {0,46.367574f,-49.763710f,19.330644f,1.659617f};

/* M1=8.75/2;                               %机体质量

Q=diag([30 1 500 100 5000 1]);
R=diag([0.75  0.25]);                %T Tp
*/

//float K11[6] = {0,-116.082290f,178.326270f,-240.290481f,-4.829183f};
//float K12[6] = {0,31.736022f,-51.487029f,-34.744900f,-0.006218f};
//float K13[6] = {0,-60.684887f,59.161979f,-19.635961f,-23.439363f};
//float K14[6] = {0,-30.095666f,38.485794f,-31.085143f,-17.305582f};
//float K15[6] = {0,-765.498298f,791.252105f,-292.914596f,54.564062f};
//float K16[6] = {0,-27.167072f,29.996800f,-12.160067f,4.024437f};
//float K21[6] = {0,-163.013736f,165.050846f,-44.547787f,15.976060f};
//float K22[6] = {0,-23.106972f,25.776029f,-3.363328f,2.005278f};
//float K23[6] = {0,-391.774734f,402.444965f,-146.189299f,23.432869f};
//float K24[6] = {0,-291.729242f,297.099369f,-105.270518f,17.414399f};
//float K25[6] = {0,497.674120f,-489.720369f,165.612912f,122.021377f};
//float K26[6] = {0,40.736170f,-41.671697f,15.058533f,2.122199f};


//float K11[6] = {0,-144.406582f,177.227122f,-174.529283f,-22.000112f};
//float K12[6] = {0,15.261134f,-24.669397f,-33.430650f,-0.244439f};
//float K13[6] = {0,-24.491161f,24.479075f,-8.396439f,-10.494203f};
//float K14[6] = {0,31.153924f,-31.169164f,5.547010f,-17.731968f};
//float K15[6] = {0,-608.670004f,650.973437f,-252.825488f,47.125632f};
//float K16[6] = {0,-28.899881f,32.281458f,-13.451008f,3.885780f};
//float K21[6] = {0,-587.307514f,626.119356f,-233.173634f,45.576525f};
//float K22[6] = {0,-12.316947f,15.296199f,-2.967432f,2.177816f};
//float K23[6] = {0,-140.000011f,151.065684f,-58.774277f,10.108352f};
//float K24[6] = {0,-236.677333f,249.941573f,-93.776614f,15.524116f};
//float K25[6] = {0,569.821441f,-567.914809f,194.852783f,66.601096f};
//float K26[6] = {0,39.070654f,-40.525301f,14.891064f,1.495483f};

/*Q=diag([700 1 600 200 1000 1]);
R=diag([0.75  0.25]);                %T Tp*/

//float K11[6] = {0,-177.172754f,244.003795f,-294.342881f,-16.034532f};
//float K12[6] = {0,33.335275f,-54.840923f,-48.409206f,0.176163f};
//float K13[6] = {0,-77.203050f,80.577718f,-29.237653f,-24.430832f};
//float K14[6] = {0,28.196959f,-19.473117f,-11.701760f,-26.097775f};
//float K15[6] = {0,-403.269573f,454.643456f,-193.513652f,43.398502f};
//float K16[6] = {0,-26.680042f,30.528263f,-13.108170f,4.798816f};
//float K21[6] = {0,-385.309931f,442.462279f,-194.524011f,51.929938f};
//float K22[6] = {0,26.162453f,-27.100576f,10.877081f,2.214143f};
//float K23[6] = {0,-272.085306f,325.116529f,-146.765207f,28.630357f};
//float K24[6] = {0,-275.793953f,316.981917f,-136.655241f,26.451918f};
//float K25[6] = {0,580.081812f,-596.719341f,214.900195f,36.010012f};
//float K26[6] = {0,47.761256f,-51.744661f,20.318614f,0.040028f};


/*Q=diag([30 1 500 100 5000 1]);


R=diag([1  0.25]); */

//float K11[6] = {0,-115.126155f,170.649515f,-215.442495f,-4.078597f};
//float K12[6] = {0,26.253624f,-43.142734f,-31.197256f,0.052569f};
//float K13[6] = {0,-57.632354f,55.994880f,-18.498064f,-20.153449f};
//float K14[6] = {0,-31.404098f,38.446801f,-28.580708f,-14.894821f};
//float K15[6] = {0,-709.043918f,732.385808f,-271.087440f,49.302362f};
//float K16[6] = {0,-25.704048f,28.312288f,-11.460745f,3.642841f};
//float K21[6] = {0,-177.741240f,181.226780f,-53.464316f,17.184152f};
//float K22[6] = {0,-23.860072f,26.177529f,-4.260697f,2.164510f};
//float K23[6] = {0,-422.004007f,433.007528f,-157.210272f,24.613085f};
//float K24[6] = {0,-314.224232f,320.009512f,-113.857296f,18.363215f};
//float K25[6] = {0,540.518166f,-530.485443f,178.763915f,120.706549f};
//float K26[6] = {0,44.207272f,-45.090225f,16.232827f,1.999465f};


/*Q=diag([30 1 500 100 5000 1]);


R=diag([0.50  0.55]); */

//float K11[6] = {0,-95.962518f,169.627359f,-273.902036f,-7.056615f};
//float K12[6] = {0,43.425716f,-67.732556f,-39.743430f,-0.232138f};
//float K13[6] = {0,-31.673164f,31.529042f,-10.847578f,-30.056454f};
//float K14[6] = {0,-4.098905f,15.508924f,-27.657956f,-22.157953f};
//float K15[6] = {0,-533.222247f,554.113928f,-207.609107f,49.649082f};
//float K16[6] = {0,-24.252486f,27.019246f,-11.051887f,4.393769f};
//float K21[6] = {0,-55.019877f,50.125849f,4.776575f,6.349756f};
//float K22[6] = {0,-13.366871f,16.699859f,0.970690f,0.745061f};
//float K23[6] = {0,-138.754164f,143.129988f,-52.182640f,10.625769f};
//float K24[6] = {0,-103.303086f,104.466842f,-35.172795f,7.670968f};
//float K25[6] = {0,159.658910f,-159.752849f,55.856381f,88.653315f};
//float K26[6] = {0,15.559312f,-16.275051f,6.107299f,2.371508f};

/*Q=diag([30 1 500 100 5000 1]);


R=diag([1  0.15]); */

//float K11[6] = {0,-140.809314f,195.584993f,-223.775760f,-3.174109f};
//float K12[6] = {0,22.615599f,-39.592911f,-32.439374f,0.193379f};
//float K13[6] = {0,-88.104246f,84.760796f,-27.508115f,-19.240943f};
//float K14[6] = {0,-54.480576f,60.286797f,-35.471894f,-14.190390f};
//float K15[6] = {0,-924.689414f,952.240364f,-350.102855f,58.344028f};
//float K16[6] = {0,-28.271728f,31.144104f,-12.608416f,3.783410f};
//float K21[6] = {0,-392.786993f,409.488705f,-147.621351f,35.233438f};
//float K22[6] = {0,-44.323355f,46.288431f,-12.961046f,4.550921f};
//float K23[6] = {0,-892.565686f,913.545194f,-330.383881f,48.239783f};
//float K24[6] = {0,-666.007821f,678.459227f,-243.314960f,36.368718f};
//float K25[6] = {0,1243.299474f,-1209.464020f,400.800893f,177.591375f};
//float K26[6] = {0,89.625919f,-90.630454f,32.130836f,1.402047f};


//float K11[6] = {0,-162.187148f,216.270592f,-230.550230f,-2.447038f};
//float K12[6] = {0,19.543109f,-36.579316f,-33.464214f,0.307723f};
//float K13[6] = {0,-112.504037f,107.699003f,-34.637975f,-18.518694f};
//float K14[6] = {0,-73.123620f,77.857552f,-40.963541f,-13.630252f};
//float K15[6] = {0,-1069.845191f,1100.340868f,-403.219555f,64.530995f};
//float K16[6] = {0,-29.497836f,32.552682f,-13.204497f,3.860381f};
//float K21[6] = {0,-661.668046f,696.234013f,-266.892586f,57.634630f};
//float K22[6] = {0,-69.351559f,71.093456f,-24.116655f,7.517234f};
//float K23[6] = {0,-1478.574422f,1511.651845f,-545.500625f,77.286567f};
//float K24[6] = {0,-1104.401855f,1125.176287f,-404.528332f,58.558440f};
//float K25[6] = {0,2184.498129f,-2115.195361f,694.850341f,237.155084f};
//float K26[6] = {0,145.315526f,-146.346591f,51.496879f,0.448983f};


/*Q=diag([700 1 500 100 1000 1]);


R=diag([1  0.15]); */

//float K11[6] = {0,-406.756010f,467.928501f,-344.969122f,-3.847140f};
//float K12[6] = {0,14.165994f,-31.208072f,-48.786503f,1.013945f};
//float K13[6] = {0,-209.660558f,210.307292f,-71.941290f,-15.894646f};
//float K14[6] = {0,-111.919959f,117.217527f,-54.628260f,-17.630240f};
//float K15[6] = {0,-461.738776f,528.519989f,-227.525254f,47.193752f};
//float K16[6] = {0,-17.394887f,21.964404f,-10.596334f,4.227899f};
//float K21[6] = {0,-1187.443507f,1425.762913f,-695.907496f,167.533073f};
//float K22[6] = {0,105.992933f,-108.178645f,23.622345f,9.136050f};
//float K23[6] = {0,-1313.790343f,1493.817460f,-629.906214f,106.757549f};
//float K24[6] = {0,-1350.684312f,1495.388495f,-612.772124f,102.585410f};
//float K25[6] = {0,2674.808137f,-2705.672997f,946.325533f,23.451652f};
//float K26[6] = {0,222.890587f,-234.839528f,88.184310f,-6.764370f};


/*Q=diag([700 1 500 100 2000 1]);


R=diag([0.75  0.25]); */

//float K11[6] = {0,-142.060165f,174.380525f,-172.864002f,-22.070548f};
//float K12[6] = {0,15.201878f,-24.654712f,-33.468257f,-0.245084f};
//float K13[6] = {0,-24.220330f,24.198181f,-8.305159f,-10.503626f};
//float K14[6] = {0,31.420024f,-31.472799f,5.660416f,-17.789930f};
//float K15[6] = {0,-605.231189f,647.117263f,-251.482252f,46.946592f};
//float K16[6] = {0,-28.823892f,32.191900f,-13.422359f,3.885122f};
//float K21[6] = {0,-584.856922f,623.309863f,-232.254994f,45.432485f};
//float K22[6] = {0,-12.429399f,15.407085f,-2.998481f,2.181621f};
//float K23[6] = {0,-139.529150f,150.454851f,-58.540657f,10.079518f};
//float K24[6] = {0,-236.223116f,249.329205f,-93.569797f,15.511948f};
//float K25[6] = {0,562.375251f,-560.392741f,192.455000f,66.824909f};
//float K26[6] = {0,38.806676f,-40.243752f,14.800316f,1.503788f};


//float K11[6] = {0,-204.518038f,244.190236f,-192.502303f,-20.983470f};
//float K12[6] = {0,8.681410f,-15.001398f,-32.622484f,-0.291806f};
//float K13[6] = {0,-24.723168f,25.894440f,-9.524138f,-10.202480f};
//float K14[6] = {0,24.960678f,-23.688762f,2.771719f,-17.150615f};
//float K15[6] = {0,-540.608305f,600.988001f,-248.521875f,52.129058f};
//float K16[6] = {0,-32.525285f,37.217660f,-16.136970f,4.825964f};
//float K21[6] = {0,-446.454088f,496.104455f,-195.405355f,48.190472f};
//float K22[6] = {0,2.065086f,0.491995f,2.830248f,2.058485f};
//float K23[6] = {0,-101.234984f,117.657275f,-50.925424f,10.641662f};
//float K24[6] = {0,-179.909086f,201.036002f,-82.131279f,16.252138f};
//float K25[6] = {0,642.351689f,-657.179151f,235.302598f,61.062266f};
//float K26[6] = {0,46.638865f,-50.056073f,19.426895f,1.648479f};


/*Q=diag([2000 10 500 100 4000 1]);


R=diag([0.75  0.25]); */
float K11[6] = {0,-370.394524f,396.471948f,-215.492650f,-37.402430f};
float K12[6] = {0,-0.976649f,-5.999054f,-31.242243f,-2.247988f};
float K13[6] = {0,-9.510301f,10.962664f,-4.499063f,-7.439460f};
float K14[6] = {0,46.799150f,-47.950495f,14.987762f,-18.613368f};
float K15[6] = {0,-647.986793f,756.504986f,-330.206225f,70.939827f};
float K16[6] = {0,-35.457577f,40.981242f,-18.049323f,5.191204f};
float K21[6] = {0,-666.064872f,799.817927f,-351.408960f,82.929940f};
float K22[6] = {0,-28.285312f,39.153869f,-15.490739f,5.697750f};
float K23[6] = {0,-29.231517f,43.917458f,-23.851837f,6.262248f};
float K24[6] = {0,-108.814958f,138.755399f,-64.980282f,14.724146f};
float K25[6] = {0,878.338377f,-916.630214f,335.715540f,83.469056f};
float K26[6] = {0,39.826487f,-44.262430f,17.953752f,2.475768f};
//========================================================================================
float K_3508_t=   2700.6667f;//
//K_3508_t = 2,730.6666666;
//=========================================================================================


/*
启用一个PID控制的流程
1.定义PID参数数组float  PID_xxx_param[7] = {KP,KI,KD,Max_I,Max_Out,Dead_Band,Lpf_Cutoff_Freq};
2.定义PID控制器PID_Info_TypeDef  PID_xxx;
3.在Control_Init()函数中调用PID初始化函数，例如：PID_xxx.PID_Param_Init(&PID_xxx,PID_xxx_param);
4.在需要使用PID控制的地方，设置目标值和测量值，例如：PID_xxx.Target = xxx; PID_xxx.Measure = xxx;
5.调用PID计算函数，例如：PID_Calc(&PID_xxx);



*/
//static float  PID_Leg_Length_F_Param[7] 	  = {1500.f,1.f ,200000.f,0.f ,0  ,10.f,200.f}; //腿长PID(change)
static float  PID_Leg_Length_F_Param[7] 	  = {1500.f,1.f ,20000.f,0.f ,0.f  ,10.f,200.f}; //腿长PID(change
static float  PID_Yaw_P_pama[7] 			      = {4.4f  ,0.f ,60.f    ,0   ,0  ,200 ,500  }; // 偏航角位置PID
static float  PID_Yaw_V_pama[7] 			      = {0.25f ,0   ,0.4f    ,0   ,0  ,200 ,70   }; // 偏航角速度PID
static float  PID_Leg_Coordinate_param[7]   = {300.f ,0.f ,20.0f   ,0.f ,0.f,0.f ,50   }; // 腿部协调PID
//由于驱动轮减速箱的机械设计存在问题，两边减速箱的阻力不同，导致输出相同的力矩，左右轮速度不同
//通过调整PID参数来尽量补偿这个问题
//static float  PID_Stool_param[2]                  = {15.f,0.f,0.f,0.f,0.f,2000.f,10000.f};//驱动轮速度控制PID

PID_Info_TypeDef PID_Leg_Coordinate;  // 腿部协调控制器
PID_Info_TypeDef PID_Leg_length_F[2]; // 左右腿长度控制器
PID_Info_TypeDef PID_Yaw[2];          // 偏航控制器（位置+速度）
PID_Info_TypeDef PID_Stool[2]; // 驱动轮控制器

//调试区----------------------------------------------------------------------------------------------


float Test_Theta = 0;                           //测试用的摆杆倾角--0度--摆杆垂直地面

//float Joint_Angle_offset_Num = 0.635f;          //机械安装偏移补偿(并联腿)

float Test_Vmc_Target_L0_Chassis_High   = 0.30f;//高底盘
float Test_Vmc_Target_L0_Chassis_Normal = 0.20f;//正常高度底盘

Control_Info_Typedef Control_Info ={
//腿的机械参数，固定值--如果是新车，这一块要改
.L_Leg_Info = {

	.Thigh_Comp_Angle = 0.0f,//大腿摆角补偿
	.Calf_Comp_Angle  = 0.0f,//小腿摆角补偿
	.Biased = {
		.L_Thigh_Link = 0.11760f,//大腿连杆，连杆直驱--动力长后杆长度（AH）
		.L_Calf_Link  = 0.098f,//小腿连杆(仿生机器人领域标准术语）动力短前杆长度（AD）,连杆传动
		.K = 0.4558139f,//连杆长度比K=AD/AH

		
	},//偏置并联
	.Gravity_Compensation = 42.f,//重力的一半
	//.Link_Gravity_Compensation_Angle = 0.4701917f,//连杆重心补偿角度
	.Link_Gravity_Compensation_Angle = 0.0f,//连杆重心补偿角度
	.Phi_Comp_Angle =  0.007f,//当车平衡时，陀螺仪读出来的数据
//.Phi_Comp_Angle =  0.0f,//当车平衡时，陀螺仪读出来的数据
},//单腿测试



//右腿
.R_Leg_Info = {
	.Thigh_Comp_Angle = 0.0f,//大腿摆角补偿
	.Calf_Comp_Angle  = 0.0f,//小腿摆角补偿
	.Biased = {
		.L_Thigh_Link = 0.11760f,//大腿连杆，连杆直驱--动力长后杆长度（AH）
		.L_Calf_Link  = 0.098f,//小腿连杆(仿生机器人领域标准术语）动力短前杆长度（AD）,连杆传动
		.K = 0.4558139f,//连杆长度比K=AD/AH

		
	},//2
	.Gravity_Compensation = 42.f,
    .Link_Gravity_Compensation_Angle = 0.4701917f,
.Phi_Comp_Angle =  0.007f,//当车平衡时，陀螺仪读出来的数据
	//.Phi_Comp_Angle =  0.0f,//当车平衡时，陀螺仪读出来的数据
},




};   


//函数区-------------------------------------------------------------------------------------------
static void Control_Init(Control_Info_Typedef *Control_Info);
/*清零所有状态量/设置默认PID参数/配置初始工作模式/分配内存资源/系统启动时调用一次*/
static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info);//1.检查电池（低电压警告）
static void Mode_Update(Control_Info_Typedef *Control_Info);           //2.模式更新
static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info);    //3.看腿的形状（读关节角度，并映射到VMC坐标系）
static void VMC_Calculate(Control_Info_Typedef *Control_Info);         //4.知道形状后，连杆正运动学解算，求简化腿腿长L0和摆角Phi0，轮子速度
static void LQR_K_Update(Control_Info_Typedef *Control_Info);          //5.得到简化腿长后，更新LQR控制器的增益矩阵K

static void Measure_Update(Control_Info_Typedef *Control_Info);        //6.更新传感器数据和目标值
static void Target_Update(Control_Info_Typedef *Control_Info);
static void LQR_X_Update(Control_Info_Typedef *Control_Info);			//7.更新状态向量差值，为LQR控制器u = -K·X提供输入向量（值）
static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info);//8.求简化腿的顶力（F）和扭矩(Tp)

static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info);		//9.自适应的LQR，根据是否在空中来限制LQR的作用

static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info);//10.综合计算
static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info);//11.实际向电机发送数据的计算



TickType_t Control_Task_SysTick = 0;


void Control_Task(void const * argument)
{
  /*只跑一次的*/
  
 	Control_Init(&Control_Info);
	
 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();
		//电池
		//Check_Low_Voltage_Beep(&Control_Info);
		//开关
		//上级函数
        Mode_Update(&Control_Info);
		//K
		//传感器-----------------------
		Joint_Angle_Offset(&Control_Info);
		//解码器
		//从真实形状转化为简化模型，方便之后的目标控制
	    VMC_Calculate(&Control_Info);
		//上级函数--------------------------------
		 //转化到二阶简化模型之后，按照二阶独轮车的模型进行控制
		LQR_K_Update(&Control_Info);
		//传感器函数
		//测量与目标

	    Measure_Update(&Control_Info);
		//上级函数
		Target_Update(&Control_Info);
		//简化模型的目标控制完成之后，开始为转化到真实的复杂小车结构做准备
		//整个过程有点像解码器和编译器
		//解码器把来自真实世界的模型数据转化为简化模型的数据，然后在上位机编写控制逻辑算法，编写完成之后，
		//解码器把简化模型的数据转化为真实世界的数据
		 //解码器函数
		//力
	      VMC_Measure_F_Tp_Calculate(&Control_Info);	
        LQR_X_Update(&Control_Info);
		LQR_T_Tp_Calculate(&Control_Info);
		Comprehensive_F_Calculate(&Control_Info);
		//输出
		Joint_Tourgue_Calculate(&Control_Info);
         //USART_Vofa_Justfloat_Transmit(Control_Info.R_Leg_Info.Measure.Chassis_Velocity,Control_Info.L_Leg_Info.Measure.Chassis_Velocity,0,0);
//USART_Vofa_Justfloat_Transmit(-INS_Info.Angle[2],INS_Info.Yaw_Angle,INS_Info.Roll_Angle,INS_Info.Roll_Angle);
  //USART_Vofa_Justfloat_Transmit(Control_Info.L_Leg_Info.Measure.Chassis_Position,Control_Info.L_Leg_Info.Measure.Phi,Control_Info.L_Leg_Info.Measure.Theta,Control_Info.L_Leg_Info.Measure.F);
	
//	USART_Vofa_Justfloat_Transmit((Control_Info.L_Leg_Info.Measure.Phi *RadiansToDegrees),Control_Info.L_Leg_Info.Moment.Balance_Tp,Control_Info.L_Leg_Info.SendValue.Current,(Control_Info.L_Leg_Info.Measure.Theta *RadiansToDegrees));
	USART_Vofa_Justfloat_Transmit((Control_Info.L_Leg_Info.Measure.Phi *RadiansToDegrees),Control_Info.L_Leg_Info.Moment.Balance_Tp,Control_Info.L_Leg_Info.SendValue.Current,Control_Info.L_Leg_Info.F);
	
		osDelayUntil(&Control_Task_SysTick,1);
  }
}
  /* USER CODE END Control_Task */
 static uint32_t Tick = 0;
//不用改，串并联通用======================================================================================
static void Check_Low_Voltage_Beep(Control_Info_Typedef *Control_Info){
 
	 Control_Info->VDC = USER_ADC_Voltage_Update();
 	
	 if(Control_Info->VDC < 22.f){//如果电压低于22V
	    Tick++;
		// 触发蜂鸣器报警模式（0.1s开/0.1s关，循环3次）
		 if(Tick < 100){
		    TIM12->CCR2 = 1000;
		 }else if(Tick > 100 && Tick <200){
		 	TIM12->CCR2 = 0;
		 }else if(Tick > 200 && Tick < 300){
		 	TIM12->CCR2 = 1000;
		 }else if(Tick > 300 && Tick < 400){
			TIM12->CCR2 = 0;
		 }else if(Tick > 400 && Tick < 500){
			TIM12->CCR2 = 1000;
		 }else if(Tick > 500 && Tick < 600){
			TIM12->CCR2 = 0;
     }else if(Tick > 1000){
			 Tick = 0;
		 }
		}else if(Control_Info->VDC >= 22.f){
	 
			TIM12->CCR2 = 0;
			
	  }
  }
//PID初始化
static void Control_Init(Control_Info_Typedef *Control_Info){

    //驱动轮PID初始化
	//PID_Init(&PID_Stool[0],PID_POSITION,PID_Stool_param);
	//PID_Init(&PID_Stool[1],PID_POSITION,PID_Stool_param);

	//腿长PID初始化
    PID_Init(&PID_Leg_length_F[0],PID_POSITION,PID_Leg_Length_F_Param);
    PID_Init(&PID_Leg_length_F[1],PID_POSITION,PID_Leg_Length_F_Param);
	// 初始化偏航PID控制器（位置环+速度环）
	PID_Init(&PID_Yaw[0],PID_POSITION,PID_Yaw_P_pama);
	PID_Init(&PID_Yaw[1],PID_POSITION,PID_Yaw_V_pama);
	// 初始化腿部协调PID控制器
	PID_Init(&PID_Leg_Coordinate, PID_POSITION, PID_Leg_Coordinate_param);
}


static void Mode_Update(Control_Info_Typedef *Control_Info){


   if(remote_ctrl.rc.s[1] == 3 || remote_ctrl.rc.s[1]){
		 /*逻辑冗余：由于||(或运算符)的存在，当remote_ctrl.rc.s[1] == 3成立时，第二个条件不会被执行（短路求值）*/
	 
	   Control_Info->Init.IF_Begin_Init = 1;
	 if(remote_ctrl.rc.s[1] == 2){
	 Control_Info->Init.IF_Begin_Init = 0;
	 Control_Info->Chassis_Situation = CHASSIS_WEAK;
	 }
	 
	 }else{
	 
	  	Control_Info->Init.IF_Begin_Init = 0;
      if(Control_Info->Chassis_Situation == CHASSIS_BALANCE) Control_Info->Chassis_Situation = CHASSIS_WEAK; 
	 
	 } 
		 

   if(Control_Info->Init.IF_Begin_Init == 1 && Control_Info->Chassis_Situation == CHASSIS_WEAK){	
	
	        
   if(Control_Info->Init.Joint_Init.IF_Joint_Init == 0){  
	   	
			 if(DM_8009_Motor[0].Data.Position < 0.0f )  
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
			 if(DM_8009_Motor[1].Data.Position > -0.21f &&  DM_8009_Motor[1].Data.Position<-0.05f)    
		   Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0; 
			 if(DM_8009_Motor[2].Data.Position < 0.20f&& DM_8009_Motor[2].Data.Position>0.10f)    
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
			 if(DM_8009_Motor[3].Data.Position > -0.0f )  
			 Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 1; else Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
			
			 if(Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] 
			  + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] + Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] == 4){  
			
			   Control_Info->Init.Joint_Init.IF_Joint_Init = 1;  
					
				}else  Control_Info->Init.Joint_Init.IF_Joint_Init =0;
					
					Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
          Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 

		}else if(Control_Info->Init.Joint_Init.IF_Joint_Init == 1){
			
			 Control_Info->Chassis_Situation = CHASSIS_BALANCE;
		   	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
        Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
				  
		}
				
}

    if(Control_Info->Init.IF_Begin_Init == 0 && Control_Info->Chassis_Situation == CHASSIS_WEAK){

  
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[0] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[1] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[2] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Reduction_Flag[3] = 0;
			Control_Info->Init.Joint_Init.IF_Joint_Init = 0;


    }
}






//确定连杆腿的形状（读取关节电机的弧度值，速度）
//打一个补丁，在这个函数顺便再读取关节电机的扭矩，方便分清各电机对应腿部什么部位
static void Joint_Angle_Offset(Control_Info_Typedef *Control_Info){
//偏置=====================================================================================================
/*
我们这样设计:
1：以逆时针为正方向，水平向右为X轴，垂直向上为Y轴，
为了方便，将机器人的腿部连杆结构放在一二象限内

2：电机与连杆是通过链条连接的，且，连杆上存在机械限位
在之后的控制中，我们需要知道连杆和电机角度（弧度值）的映射关系，
在这里，为了方便，我们先将连杆转到最低的限位，（蹲在地上），
然后重置关节电机的零点。
3：
（不同与并联腿，偏置腿可以全周转动，
也就是说，并联腿的连杆活动范围是固定的（比如无法转到头上），
而偏置腿上的限位只是限制了大小连杆的相对角度，大小连杆本身可以自由转动，
这导致一个问题，
也就是给关节电机定零点时，
你没法完全保证大小连杆各自和水平线的夹角是相等的
故定零点时要注意，
要么摆姿势让夹角相等，
要么定好零点之后分别计算大小连杆和关节电机零点的相对角度（弧度制））
4：要求什么
左右腿大小动力杆（共四根）与各自对应的关节电机（共四个）之间的映射关系
在求之前，先要完成的处理是
（0.1）设置好关节电机的ID和模式
（0.2）配对大小动力杆和关节电机
（1）已经合理的重置好了关节电机的零点，重置零点时，最好书面记入一下当时机器人腿部的姿势
（2）重置好零点后，记入大小动力杆转到水平线时，关节电机的弧度值，
5：开始映射

*/
//左腿
	// //左小腿摆角与0号关节电机的映射(摆角=补偿角度+电机角度)
	// Control_Info->L_Leg_Info.Biased.Calf_Angle 		 	= Control_Info->L_Leg_Info.Calf_Comp_Angle	   +(DM_8009_Motor[0].Data.Position);
	// //左大腿摆角与1号关节电机的映射（摆角=PI-补偿角度+电机角度）
	// Control_Info->L_Leg_Info.Biased.Thigh_Angle 		 = (PI-Control_Info->L_Leg_Info.Thigh_Comp_Angle)+(DM_8009_Motor[1].Data.Position);

//测试用：假设，当腿完全收缩时-也就是到达最低机械限位，时，大腿和小腿连杆可以张到180度（实际是180.03度）

    // //左小腿摆角与0号关节电机的映射(摆角=电机角度)
	 Control_Info->L_Leg_Info.Biased.Calf_Angle 		 =  DM_8009_Motor[0].Data.Position ;//* RadiansToDegrees;
	// //左大腿摆角与1号关节电机的映射（摆角=电机角度）
	 Control_Info->L_Leg_Info.Biased.Thigh_Angle 		 =  PI+ DM_8009_Motor[1].Data.Position;//) * RadiansToDegrees;
	//角速度
		//左大腿
		Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot = DM_8009_Motor[1].Data.Velocity;
		//左小腿
		Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot  = DM_8009_Motor[0].Data.Velocity;
	//力矩映射
		//左大腿--对应T1
		Control_Info->L_Leg_Info.Biased.T_Thigh = DM_8009_Motor[1].Data.Torque;
		//左小腿---对应T2
		Control_Info->L_Leg_Info.Biased.T_Calf  = DM_8009_Motor[0].Data.Torque;
//右腿
	// //右大腿摆角和2号关节电机的映射(同左腿)
	// Control_Info->R_Leg_Info.Biased.Thigh_Angle 		 = (PI -Control_Info->R_Leg_Info.Thigh_Comp_Angle)+(DM_8009_Motor[2].Data.Position);
	// //右小腿摆角和3号关节电机的映射
	// Control_Info->R_Leg_Info.Biased.Calf_Angle  		 = Control_Info->R_Leg_Info.Calf_Comp_Angle		+(DM_8009_Motor[3].Data.Position);



	//测试用
	//测试用：假设，当腿完全收缩时-也就是到达最低机械限位，时，大腿和小腿连杆可以张到180度（实际是180.03度）

	// //右大腿摆角和2号关节电机的映射(同左腿)
	 Control_Info->R_Leg_Info.Biased.Thigh_Angle 		 =  DM_8009_Motor[2].Data.Position ;//* RadiansToDegrees;
	// //右小腿摆角和3号关节电机的映射
	 Control_Info->R_Leg_Info.Biased.Calf_Angle  		 =  PI+ DM_8009_Motor[3].Data.Position ;//* RadiansToDegrees;
	//角速度
		//右大腿
		Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot = DM_8009_Motor[2].Data.Velocity;
		//右小腿
		Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot  = DM_8009_Motor[3].Data.Velocity;
	//力矩映射
		//右大腿--对应T2
		Control_Info->R_Leg_Info.Biased.T_Thigh = DM_8009_Motor[2].Data.Torque;
		//右小腿---对应T1
		Control_Info->R_Leg_Info.Biased.T_Calf  = DM_8009_Motor[3].Data.Torque;

//  //五连杆并联结构--------------------------------------------------------------------------------------------------
// 	//我们这样设计：设关节电机在下限位处为电机的零点
//  //左腿
// 	//连杆角度
//  	Control_Info->L_Leg_Info.VMC.Phi1 		= (PI + Joint_Angle_offset_Num)+(DM_8009_Motor[0].Data.Position);
//  	Control_Info->L_Leg_Info.VMC.Phi4 		= DM_8009_Motor[1].Data.Position - Joint_Angle_offset_Num;
// 	//电机速度
// 	Control_Info->L_Leg_Info.VMC.Phi1_dot 	= DM_8009_Motor[0].Data.Velocity;
// 	Control_Info->L_Leg_Info.VMC.Phi4_dot 	= DM_8009_Motor[1].Data.Velocity;
// //右腿
// 	//连杆角度
// 	  Control_Info->R_Leg_Info.VMC.Phi1 	= (3.141593f + 0.635f) + (DM_8009_Motor[3].Data.Position );
//       Control_Info->R_Leg_Info.VMC.Phi4 	= DM_8009_Motor[2].Data.Position - 0.635f;
// 	 //速度 
// 	  Control_Info->R_Leg_Info.VMC.Phi1_dot = DM_8009_Motor[3].Data.Velocity;
// 	  Control_Info->R_Leg_Info.VMC.Phi4_dot = DM_8009_Motor[2].Data.Velocity;

}

//更新一下
/*
整个运控的控制思路是：


*/
/*形状确定，开始计算其简化模型
--求解虚拟腿长L0，为了更新不同腿长下的雅可比矩阵

求解虚拟腿的角度，为了更新状态空间方程

//求解轮子的X轴和Y轴上的速度（两条腿）

//最主要的改的地方*/
static void VMC_Calculate(Control_Info_Typedef *Control_Info){
//偏置并联结构===============================================================================================================================
//这个部分是将真实复杂的小车结构，转换为简化的二阶倒立摆的预处理
    //1:求解虚拟腿的角度，为了更新状态空间方程
	//2:求解虚拟腿长L0，为了更新不同腿长下的雅可比矩阵
	//3:求解轮子的X轴和Y轴上的速度（两条腿）
//左腿
    //中间变量=====================================================================================================================================
	//a,b,简化表达用
	Control_Info->L_Leg_Info.a = Control_Info->L_Leg_Info.Biased.L_Calf_Link;
	Control_Info->L_Leg_Info.b = Control_Info->L_Leg_Info.Biased.L_Thigh_Link;
	//左夹角差半值=（左小腿摆角-左大腿摆角）/2
	Control_Info->L_Leg_Info.M = (Control_Info->L_Leg_Info.Biased.Calf_Angle - Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	//左夹角和半差=（左小腿摆角+左大腿摆角）/2
	//2025.11.14：修改为负号
	//2.0：改为正号
	Control_Info->L_Leg_Info.N = (Control_Info->L_Leg_Info.Biased.Calf_Angle + Control_Info->L_Leg_Info.Biased.Thigh_Angle)/2.f;
	//s_radicand  
	//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	//M = (θ_1 - θ_2 )/2
	Control_Info->L_Leg_Info.S_Radicand = Control_Info->L_Leg_Info.b * Control_Info->L_Leg_Info.b - Control_Info->L_Leg_Info.a * Control_Info->L_Leg_Info.a * arm_sin_f32(Control_Info->L_Leg_Info.M) * arm_sin_f32(Control_Info->L_Leg_Info.M);
	//S 中间变量 公式： S = sqrt(S_Radicand)
	arm_sqrt_f32(Control_Info->L_Leg_Info.S_Radicand,&Control_Info->L_Leg_Info.S);
	//t //中间变量 公式： t=a*cosM+S
	Control_Info->L_Leg_Info.t = Control_Info->L_Leg_Info.a * arm_cos_f32(Control_Info->L_Leg_Info.M) + Control_Info->L_Leg_Info.S;
	//A //中间变量 公式： A= (a*t*sinM)/S
	Control_Info->L_Leg_Info.A = (Control_Info->L_Leg_Info.a * Control_Info->L_Leg_Info.t * arm_sin_f32(Control_Info->L_Leg_Info.M)) / Control_Info->L_Leg_Info.S;
    //=============================================================================================================================================================================================================================================

	//建模------------------------------------------------------------------------------------------------------------------------------
	//1.虚拟腿摆角
	Control_Info->L_Leg_Info.Sip_Leg_Angle  = Control_Info->L_Leg_Info.N;
	//虚拟腿长（AJ）AJ = t/K
	Control_Info->L_Leg_Info.Sip_Leg_Length = Control_Info->L_Leg_Info.t / Control_Info->L_Leg_Info.Biased.K;
    //J点速度
	//X轴方向速度 公式： X_J_Dot =(A*cosN*(Thigh_Angle_Dot -Calf_Angle_Dot) - t*sinN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
     Control_Info->L_Leg_Info.X_J_Dot =   (Control_Info->L_Leg_Info.A * arm_cos_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
									     - Control_Info->L_Leg_Info.t * arm_sin_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->L_Leg_Info.Biased.K);
	//Y轴方向速度 公式： Y_J_Dot =(A*sinN*(Thigh_Angle_Dot -Calf_Angle_Dot) + t*cosN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
     Control_Info->L_Leg_Info.Y_J_Dot =   (Control_Info->L_Leg_Info.A * arm_sin_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot)
									     + Control_Info->L_Leg_Info.t * arm_cos_f32(Control_Info->L_Leg_Info.N) *(Control_Info->L_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->L_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->L_Leg_Info.Biased.K);

//右腿---
//存在问题，即对于J点速度的计算，左右腿的结果不一样
//待实车测试验证
	//中间变量=====================================================================================================================================
	//a,b,简化表达用
	Control_Info->R_Leg_Info.a = Control_Info->R_Leg_Info.Biased.L_Calf_Link;
	Control_Info->R_Leg_Info.b = Control_Info->R_Leg_Info.Biased.L_Thigh_Link;
	//右夹角差半值=（右大腿摆角-右小腿摆角）/2
	Control_Info->R_Leg_Info.M = (Control_Info->R_Leg_Info.Biased.Thigh_Angle - Control_Info->R_Leg_Info.Biased.Calf_Angle)/2.f;
	//右夹角和半差=（右小腿摆角+右大腿摆角）/2
	Control_Info->R_Leg_Info.N = (Control_Info->R_Leg_Info.Biased.Calf_Angle + Control_Info->R_Leg_Info.Biased.Thigh_Angle)/2.f;
	//s_radicand
	//s_radicand  
	//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	//M = (θ_1 - θ_2 )/2
		//中间变量 公式： S_Radicand = b^2 - a^2 sin^2(M)
	Control_Info->R_Leg_Info.S_Radicand = Control_Info->R_Leg_Info.b * Control_Info->R_Leg_Info.b - Control_Info->R_Leg_Info.a * Control_Info->R_Leg_Info.a * arm_sin_f32(Control_Info->R_Leg_Info.M) * arm_sin_f32(Control_Info->R_Leg_Info.M);
	//S 中间变量 公式： S = sqrt(S_Radicand)
	arm_sqrt_f32(Control_Info->R_Leg_Info.S_Radicand,&Control_Info->R_Leg_Info.S);
	//t //中间变量 公式： t=a*cosM+S
	Control_Info->R_Leg_Info.t = Control_Info->R_Leg_Info.a * arm_cos_f32(Control_Info->R_Leg_Info.M) + Control_Info->R_Leg_Info.S;
	//A //中间变量 公式： A= (a*t*sinM)/S
	Control_Info->R_Leg_Info.A = (Control_Info->R_Leg_Info.a * Control_Info->R_Leg_Info.t * arm_sin_f32(Control_Info->R_Leg_Info.M)) / Control_Info->R_Leg_Info.S;
	//=============================================================================================================================================================================================================================================	
    //建模
	//1.虚拟腿摆角
	Control_Info->R_Leg_Info.Sip_Leg_Angle  = Control_Info->R_Leg_Info.N;
	//虚拟腿长（AJ）AJ = t/K
	Control_Info->R_Leg_Info.Sip_Leg_Length = Control_Info->R_Leg_Info.t / Control_Info->R_Leg_Info.Biased.K;
	//J点速度
	//速度有方向，取小车头前方向为正方向，垂直地面的法向量方向为单独分析单腿速度的Y轴正方向
	//X轴方向速度 公式： X_J_Dot =(A*cosN*(Thigh_Angle_Dot -Calf_Angle_Dot) - t*sinN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
	Control_Info->R_Leg_Info.X_J_Dot =   (Control_Info->R_Leg_Info.A * arm_cos_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
									     - Control_Info->R_Leg_Info.t * arm_sin_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->R_Leg_Info.Biased.K);
	//Y轴方向速度 公式： Y_J_Dot =(A*sinN*(Thigh_Angle_Dot -Calf_Angle_Dot) + t*cosN*(Thigh_Angle_Dot +Calf_Angle_Dot))/ (2*K)
	Control_Info->R_Leg_Info.Y_J_Dot =   (Control_Info->R_Leg_Info.A * arm_sin_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot - Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot)
									     + Control_Info->R_Leg_Info.t * arm_cos_f32(Control_Info->R_Leg_Info.N) *(Control_Info->R_Leg_Info.Biased.Thigh_Angle_Dot + Control_Info->R_Leg_Info.Biased.Calf_Angle_Dot))/ (2.f * Control_Info->R_Leg_Info.Biased.K);



}

//根据简化后的腿长，更新增益矩阵K----不用改
static void LQR_K_Update(Control_Info_Typedef *Control_Info){
	//简化表达
	float L_L0; 
	//L_L0= Control_Info->L_Leg_Info.VMC.L0;(并腿)
L_L0= Control_Info->L_Leg_Info.Sip_Leg_Length;//偏腿

/*左腿LQR增益计算（六状态量）计算每个增益元素的公式为：
//K = a * L0^3 + b * L0^2 + c * L0 + d

powf(变量，指数)：eg powf(L_L0,3)表示L_L0的立方

其中，a, b, c, d 是预先通过拟合得到的系数，这些系数存储在数组K11, K12, ..., K26中。
注意，这些数组有6个元素（索引0到5），但这里只用到了索引1到4（即K11[1]到K11[4]），
而索引0未使用（从代码中看，K11[0]被初始化为0，但没有被使用）
。
这种设计使LQR控制器能够自适应腿长变化​：
1. 当腿伸长时（L0增大），系统惯性矩增加，需要更大的控制增益
2. 当腿缩短时（L0减小），系统响应更快，需要减小控制增益
3. 多项式系数通过系统辨识和优化算法预先确定，确保在全工作范围内保持最优性能
这种自适应策略显著提高了双足机器人在不同工作状态（如行走、站立、抬腿）下的控制稳定性和鲁棒性。
*/	
	Control_Info->L_Leg_Info.LQR_K[0][0] =   K11[1]*powf(L_L0,3)   + K11[2]*powf(L_L0,2)    +K11[3]*L_L0    +K11[4];      
	Control_Info->L_Leg_Info.LQR_K[0][1] =   K12[1]*powf(L_L0,3)   + K12[2]*powf(L_L0,2)    +K12[3]*L_L0    +K12[4];
	Control_Info->L_Leg_Info.LQR_K[0][2] =   K13[1]*powf(L_L0,3)   + K13[2]*powf(L_L0,2)    +K13[3]*L_L0    +K13[4];
	Control_Info->L_Leg_Info.LQR_K[0][3] =   K14[1]*powf(L_L0,3)   + K14[2]*powf(L_L0,2)    +K14[3]*L_L0    +K14[4];
	Control_Info->L_Leg_Info.LQR_K[0][4] =   K15[1]*powf(L_L0,3)   + K15[2]*powf(L_L0,2)    +K15[3]*L_L0    +K15[4];
	Control_Info->L_Leg_Info.LQR_K[0][5] =   K16[1]*powf(L_L0,3)   + K16[2]*powf(L_L0,2)    +K16[3]*L_L0    +K16[4];

	Control_Info->L_Leg_Info.LQR_K[1][0] =   K21[1]*powf(L_L0,3)   + K21[2]*powf(L_L0,2)    +K21[3]*L_L0    +K21[4];     
	Control_Info->L_Leg_Info.LQR_K[1][1] =   K22[1]*powf(L_L0,3)   + K22[2]*powf(L_L0,2)    +K22[3]*L_L0    +K22[4];
	Control_Info->L_Leg_Info.LQR_K[1][2] =   K23[1]*powf(L_L0,3)   + K23[2]*powf(L_L0,2)    +K23[3]*L_L0    +K23[4];
	Control_Info->L_Leg_Info.LQR_K[1][3] =   K24[1]*powf(L_L0,3)   + K24[2]*powf(L_L0,2)    +K24[3]*L_L0    +K24[4];
	Control_Info->L_Leg_Info.LQR_K[1][4] =   K25[1]*powf(L_L0,3)   + K25[2]*powf(L_L0,2)    +K25[3]*L_L0    +K25[4];
	Control_Info->L_Leg_Info.LQR_K[1][5] =   K26[1]*powf(L_L0,3)   + K26[2]*powf(L_L0,2)    +K26[3]*L_L0    +K26[4];
//右腿		
	 float R_L0;
	 //R_L0= Control_Info->R_Leg_Info.VMC.L0;(并腿)
	 R_L0= Control_Info->R_Leg_Info.Sip_Leg_Length;//偏腿
	
	 Control_Info->R_Leg_Info.LQR_K[0][0] =   K11[1]*powf(R_L0,3)   + K11[2]*powf(R_L0,2)    +K11[3]*R_L0    +K11[4];      
	 Control_Info->R_Leg_Info.LQR_K[0][1] =   K12[1]*powf(R_L0,3)   + K12[2]*powf(R_L0,2)    +K12[3]*R_L0    +K12[4];
	 Control_Info->R_Leg_Info.LQR_K[0][2] =   K13[1]*powf(R_L0,3)   + K13[2]*powf(R_L0,2)    +K13[3]*R_L0    +K13[4];
	 Control_Info->R_Leg_Info.LQR_K[0][3] =   K14[1]*powf(R_L0,3)   + K14[2]*powf(R_L0,2)    +K14[3]*R_L0    +K14[4];
	 Control_Info->R_Leg_Info.LQR_K[0][4] =   K15[1]*powf(R_L0,3)   + K15[2]*powf(R_L0,2)    +K15[3]*R_L0    +K15[4];
	 Control_Info->R_Leg_Info.LQR_K[0][5] =   K16[1]*powf(R_L0,3)   + K16[2]*powf(R_L0,2)    +K16[3]*R_L0    +K16[4];

	 Control_Info->R_Leg_Info.LQR_K[1][0] =   K21[1]*powf(R_L0,3)   + K21[2]*powf(R_L0,2)    +K21[3]*R_L0    +K21[4];     
	 Control_Info->R_Leg_Info.LQR_K[1][1] =   K22[1]*powf(R_L0,3)   + K22[2]*powf(R_L0,2)    +K22[3]*R_L0    +K22[4];
	 Control_Info->R_Leg_Info.LQR_K[1][2] =   K23[1]*powf(R_L0,3)   + K23[2]*powf(R_L0,2)    +K23[3]*R_L0    +K23[4];
	 Control_Info->R_Leg_Info.LQR_K[1][3] =   K24[1]*powf(R_L0,3)   + K24[2]*powf(R_L0,2)    +K24[3]*R_L0    +K24[4];
	 Control_Info->R_Leg_Info.LQR_K[1][4] =   K25[1]*powf(R_L0,3)   + K25[2]*powf(R_L0,2)    +K25[3]*R_L0    +K25[4];
	 Control_Info->R_Leg_Info.LQR_K[1][5] =   K26[1]*powf(R_L0,3)   + K26[2]*powf(R_L0,2)    +K26[3]*R_L0    +K26[4];


}




/**
  * @brief 更新传感器测量值
  * @param Control_Info 控制信息结构体
  * 
  * 融合IMU、电机编码器等数据，计算关键状态
  * 陀螺仪数据来自底盘的达妙开发板，底盘的倾斜角和角速度
  * 
  * 主要步骤包括：

1.从IMU读取底盘倾斜角和角速度，并转换到左右腿结构体。
2.计算左右腿虚拟腿的倾斜角（Theta）和倾斜角速度（Theta_dot）。
3.计算轮速、角速度、线速度。
4.计算虚拟腿长的变化率（L0_dot）。
5.估计底盘速度（Body）并进行滤波融合（Predict和Fusion）。
6.计算整体底盘速度（取左右腿速度平均值）。
7.根据目标速度更新底盘位置（如果目标速度为0则积分，否则重置为0）。
8.计算底盘加速度（考虑向心加速度和重力分量）。
9.如果底盘处于虚弱状态（未平衡），则重置所有状态为0。
  */
static void Measure_Update(Control_Info_Typedef *Control_Info){
//填状态
	//左腿
		//身体平衡
	//机体水平倾角
		Control_Info->L_Leg_Info.Measure.Phi       = -INS_Info.Angle[2]-   Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
	//Control_Info->L_Leg_Info.Measure.Phi       = INS_Info.Angle[2]+   Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
	
		Control_Info->L_Leg_Info.Measure.Phi_dot   = -INS_Info.Gyro[0];
		//腿的姿势
		//Control_Info->L_Leg_Info.Measure.Theta     = 	((PI/2) - Control_Info->L_Leg_Info.VMC.Phi0) - Control_Info->L_Leg_Info.Measure.Phi;
		Control_Info->L_Leg_Info.Measure.Theta     =     (	((PI/2) - Control_Info->L_Leg_Info.Sip_Leg_Angle) - Control_Info->L_Leg_Info.Measure.Phi);// +(PI/6.0f);//+ Control_Info->L_Leg_Info.Link_Gravity_Compensation_Angle;
		
		//Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.VMC.X_C_dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
		//											  		   +  Control_Info->L_Leg_Info.VMC.Y_C_dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
		//											  		   /  Control_Info->L_Leg_Info.VMC.L0;
		Control_Info->L_Leg_Info.Measure.Theta_dot = 			 (Control_Info->L_Leg_Info.X_J_Dot * arm_cos_f32( - Control_Info->L_Leg_Info.Measure.Theta)
													  		   +  Control_Info->L_Leg_Info.Y_J_Dot * arm_sin_f32( - Control_Info->L_Leg_Info.Measure.Theta))
													  		   /  Control_Info->L_Leg_Info.Sip_Leg_Length;
	
		//右腿
	// 右腿姿态（使用相同IMU数据）	
		Control_Info->R_Leg_Info.Measure.Phi 		  = -INS_Info.Angle[2] - Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
		//Control_Info->R_Leg_Info.Measure.Phi 		  = INS_Info.Angle[2] + Control_Info->L_Leg_Info.Phi_Comp_Angle;//注意极性
		Control_Info->R_Leg_Info.Measure.Phi_dot 	= -INS_Info.Gyro[0];
		
		Control_Info->R_Leg_Info.Measure.Theta 		= (Control_Info->R_Leg_Info.Sip_Leg_Angle -(PI/2)) - Control_Info->R_Leg_Info.Measure.Phi ;//+(PI/6.0f);//+Control_Info->R_Leg_Info.Link_Gravity_Compensation_Angle;
		Control_Info->R_Leg_Info.Measure.Theta_dot  = ( -Control_Info->R_Leg_Info.X_J_Dot     * arm_cos_f32(-Control_Info->R_Leg_Info.Measure.Theta)
                                               		+  Control_Info->R_Leg_Info.Y_J_Dot       * arm_sin_f32(-Control_Info->R_Leg_Info.Measure.Theta))
	                                             	/  Control_Info->R_Leg_Info.Sip_Leg_Length;	
	
/*
 虚拟腿倾斜角 = (π/2 - 虚拟腿安装角) - 底盘倾斜角
Theta_L = (π/2 - Phi0_L) - Phi_L;

 虚拟腿倾斜角速度 = [X方向速度·cos(Theta) + Y方向速度·sin(Theta)] / 腿长L0
Theta_dot_L = [X_C_dot·cos(-Theta_L) + Y_C_dot·sin(-Theta_L)] / L0_L;
关键变量​：
Phi0：虚拟腿安装角度（固定几何参数）
X_C_dot, Y_C_dot：足端速度（从VMC计算获得）
*/	

//3. ​轮速数据处理
/*
// 左轮转速转换（RPM → rad/s → m/s）
左轮角速度 = 电机原始数据 × (π/30) / 减速比;
左轮线速度 = 左轮角速度 × 轮半径;
*/
Control_Info->L_Leg_Info.Velocity.Wheel = Chassis_Motor[0].Data.Velocity*(PI /30.f)/15.f;
/*计算原理​
W=v wheel− ϕ˙ + θ˙ 
​轮速补偿​：减去底盘旋转引起的速度分量
​腿部运动补偿​：加上虚拟腿摆动产生的速度分量
​目标​：获得底盘在水平方向的真实运动速度*/
Control_Info->L_Leg_Info.Velocity.W     = (Control_Info->L_Leg_Info.Velocity.Wheel - Control_Info->L_Leg_Info.Measure.Phi_dot + Control_Info->L_Leg_Info.Measure.Theta_dot);
//轮半径--0.055f---x--线速度
Control_Info->L_Leg_Info.Velocity.X     = Control_Info->L_Leg_Info.Velocity.W * 0.055f;
//求腿长变化率
Control_Info->L_Leg_Info.Sip_Leg_Length_dot   = Control_Info->L_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta);
/*4. ​底盘速度融合​
采用一阶滞后滤波器融合预测值和测量值：
//估计底盘速度（Body）并进行滤波融合（Predict和Fusion）

预测速度 = 上一周期融合速度 + 加速度×Δt
融合速度 = 0.8×预测速度 + 0.2×当前测量速度---
滤波系数​：0.8（历史数据权重）和 0.2（新数据权重）
​物理意义​：在快速响应和噪声抑制之间取得平衡
*/
Control_Info->L_Leg_Info.Velocity.Body     		  = Control_Info->L_Leg_Info.Velocity.X;
//预测速度
Control_Info->L_Leg_Info.Predict_Velocity  		  = Control_Info->L_Leg_Info.Velocity.Fusion + Control_Info->Accel * 0.001f;
//融合速度
Control_Info->L_Leg_Info.Velocity.Fusion   		  = Control_Info->L_Leg_Info.Velocity.Predict* 0.8f + Control_Info->L_Leg_Info.Velocity.Body * 0.2f;
//底盘的实际速度（来自左腿测量）
Control_Info->L_Leg_Info.Measure.Chassis_Velocity = Control_Info->L_Leg_Info.Velocity.Fusion;
//右腿
  //右腿同理
	Control_Info->R_Leg_Info.Velocity.Wheel 	= -Chassis_Motor[1].Data.Velocity*(3.141593f/30.f)/15.f;

  	Control_Info->R_Leg_Info.Velocity.W     	= (Control_Info->R_Leg_Info.Velocity.Wheel - Control_Info->R_Leg_Info.Measure.Phi_dot + Control_Info->R_Leg_Info.Measure.Theta_dot);

	Control_Info->R_Leg_Info.Velocity.X     	= Control_Info->R_Leg_Info.Velocity.W * 0.055f;
	
	Control_Info->R_Leg_Info.Sip_Leg_Length_dot =  Control_Info->R_Leg_Info.Y_J_Dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta);

	Control_Info->R_Leg_Info.Velocity.Body 		=    Control_Info->R_Leg_Info.Velocity.X;
	                                          //+ Control_Info->R_Leg_Info.VMC.L0 * Control_Info->R_Leg_Info.Measure.Theta_dot * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
																						//+ Control_Info->R_Leg_Info.VMC.L0_dot * arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta);
	      
				
	Control_Info->R_Leg_Info.Velocity.Predict = Control_Info->R_Leg_Info.Velocity.Fusion + Control_Info->Accel*0.001f;	
				
	Control_Info->R_Leg_Info.Velocity.Fusion = 0.8f *	 Control_Info->R_Leg_Info.Velocity.Predict + 	Control_Info->R_Leg_Info.Velocity.Body * 0.2f;

  	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = Control_Info->R_Leg_Info.Velocity.Fusion;
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//计算整体底盘速度（取左右腿速度平均值）
  	Control_Info->Chassis_Velocity =   (Control_Info->L_Leg_Info.Measure.Chassis_Velocity +  Control_Info->R_Leg_Info.Measure.Chassis_Velocity)/2.f ;
//	7. ​底盘位置更新逻辑
	if(Control_Info->Target_Velocity == 0){//目标速度 == 0
   // 积分计算位置：位置 += 速度 × Δt
	Control_Info->L_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity*0.001f ;
	
  	Control_Info->R_Leg_Info.Measure.Chassis_Position += Control_Info->Chassis_Velocity*0.001f ;

	}else{
	    // 重置位置（速度控制模式下位置环不工作）
		  Control_Info->L_Leg_Info.Measure.Chassis_Position = 0 ;
	
    	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
	
	
	}	
//	6. ​底盘加速度计算
/*
Accel = [
    (-a_y + ω_z² × 0.155 - g × sin(Φ)) × cos(Φ)
    + (a_z - g × cos(Φ)) × sin(Φ)
]

分量解析​：
-a_y：Y轴加速度（传感器坐标系）
ω_z² × 0.155：向心加速度（0.155m为旋转半径）
g × sin(Φ)：重力分量在底盘平面投影
(a_z - g × cos(Φ)) × sin(Φ)：Z轴加速度扣除重力分量后的有效分量
*/			
 Control_Info->Accel =  (float) (( -INS_Info.Accel[1] + powf(INS_Info.Gyro[2],2)*0.155f) - GravityAccel * arm_sin_f32 (-INS_Info.Angle[2])) * arm_cos_f32 (-INS_Info.Angle[2]) + 
	                              (   INS_Info.Accel[2] - GravityAccel* arm_cos_f32 (-INS_Info.Angle[2])) * arm_sin_f32 (-INS_Info.Angle[2]) ; 		

//8. ​特殊状态处理（CHASSIS_WEAK）​​
//当底盘处于虚弱状态（未平衡状态）时：
 if(Control_Info->Chassis_Situation == CHASSIS_WEAK){
// 重置所有状态量为0 
   		Control_Info->L_Leg_Info.Measure.Phi = 0;
	 	  Control_Info->L_Leg_Info.Measure.Phi_dot = 0;
	  	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
	  	Control_Info->L_Leg_Info.Measure.Chassis_Velocity = 0;
	 	  Control_Info->L_Leg_Info.Measure.Theta = 0;
	 	  Control_Info->L_Leg_Info.Measure.Theta_dot = 0;
 
	  	Control_Info->R_Leg_Info.Measure.Phi = 0;
	  	Control_Info->R_Leg_Info.Measure.Phi_dot = 0;
	  	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0;
	  	Control_Info->R_Leg_Info.Measure.Chassis_Velocity = 0;
	  	Control_Info->R_Leg_Info.Measure.Theta = 0;
	  	Control_Info->R_Leg_Info.Measure.Theta_dot = 0;
 
	  	Control_Info->Chassis_Velocity = 0;
		//目的​：防止未初始化状态下的错误控制输出
 }



}






//功能：根据遥控器通道3（速度控制）和开关s[1]（腿长模式）更新目标值*/
//它主要处理两种控制指令：移动速度控制和腿部高度控制。
static void Target_Update(Control_Info_Typedef *Control_Info){
/*1，移动逻辑
两种模式，速度/位置模式
移动靠遥控器给速度值，不控制就自动变回位置模式，也就是在原地保持平衡，哪也不去。
*/
if(remote_ctrl.rc.ch[3] != 0 ){//开始控制
//关键参数​：
//0.031：摇杆值到速度的转换系数（满摇杆对应约±1.6 m/s）
//0.0013：加速斜坡斜率（加速响应较快）
//0.002：减速斜坡斜率（减速响应较缓
//平滑
	Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,-remote_ctrl.rc.ch[3] * 0.031,0.0013f);
//沉默位置模式
	//重置位置积分器
	//左右腿位置刷新
	Control_Info->L_Leg_Info.Measure.Chassis_Position = 0;
	Control_Info->R_Leg_Info.Measure.Chassis_Position = 0 ; 
}else if(remote_ctrl.rc.ch[3] == 0){//不移动了，开摆
	//平滑归零
	Control_Info->Target_Velocity = f_Ramp_Calc(Control_Info->Target_Velocity,0,0.002f);//0.002f--慢刹车
}
//2.约束(行车不规范，亲人两行泪)
//此处限速1.6m/s
VAL_LIMIT(Control_Info->Target_Velocity,-1.6f,1.6f);

//3.腿高控制（雄起/正常）
//雄起
if(remote_ctrl.rc.s[1] == 1){
//目标腿长0,38米
Control_Info->L_Leg_Info.Target_Sip_Leg_Length = Test_Vmc_Target_L0_Chassis_High;
//右
Control_Info->R_Leg_Info.Target_Sip_Leg_Length =  Test_Vmc_Target_L0_Chassis_High;
//正常
}else {
//目标0.17米
Control_Info->L_Leg_Info.Target_Sip_Leg_Length = Test_Vmc_Target_L0_Chassis_Normal;
 Control_Info->R_Leg_Info.Target_Sip_Leg_Length =Test_Vmc_Target_L0_Chassis_Normal;
		}


}


/*该函数实现了：
​状态反馈​：提供当前系统状态与理想状态的偏差
​控制基础​：为LQR控制器 u = -K·X 提供输入向量
​多目标协调​：平衡位置保持、速度跟踪和姿态稳定多个控制目标
​双环控制​：
内环：虚拟腿角度/速度控制（索引0-1）
外环：底盘位置/速度控制（索引2-3*/
static void LQR_X_Update(Control_Info_Typedef *Control_Info){
/*L_Leg_Info.LQR_X[0] = (目标θ - 测量θ)
L_Leg_Info.LQR_X[1] = (目标dθ/dt - 测量dθ/dt)
L_Leg_Info.LQR_X[2] = (目标位置 - 测量位置)
L_Leg_Info.LQR_X[3] = (系统目标速度 - 底盘测量速度)
L_Leg_Info.LQR_X[4] = (目标φ - 测量φ)
L_Leg_Info.LQR_X[5] = (目标dφ/dt - 测量dφ/dt)*/
//左腿
	Control_Info->L_Leg_Info.LQR_X[0] = (Control_Info->L_Leg_Info.Target.Theta            - Control_Info->L_Leg_Info.Measure.Theta);
	Control_Info->L_Leg_Info.LQR_X[1] = (Control_Info->L_Leg_Info.Target.Theta_dot        - Control_Info->L_Leg_Info.Measure.Theta_dot);
	Control_Info->L_Leg_Info.LQR_X[2] = (Control_Info->L_Leg_Info.Target.Chassis_Position - Control_Info->L_Leg_Info.Measure.Chassis_Position);
	Control_Info->L_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity);
	Control_Info->L_Leg_Info.LQR_X[4] = (Control_Info->L_Leg_Info.Target.Phi              - Control_Info->L_Leg_Info.Measure.Phi);
	Control_Info->L_Leg_Info.LQR_X[5] = (Control_Info->L_Leg_Info.Target.Phi_dot          - Control_Info->L_Leg_Info.Measure.Phi_dot);
//右腿
//右腿同理
	Control_Info->R_Leg_Info.LQR_X[0] = (Control_Info->R_Leg_Info.Target.Theta 			      - Control_Info->R_Leg_Info.Measure.Theta);
	Control_Info->R_Leg_Info.LQR_X[1] = (Control_Info->R_Leg_Info.Target.Theta_dot        - Control_Info->R_Leg_Info.Measure.Theta_dot);
	Control_Info->R_Leg_Info.LQR_X[2] = (Control_Info->R_Leg_Info.Target.Chassis_Position - Control_Info->R_Leg_Info.Measure.Chassis_Position) ;
	Control_Info->R_Leg_Info.LQR_X[3] = (Control_Info->Target_Velocity                    - Control_Info->Chassis_Velocity );
	Control_Info->R_Leg_Info.LQR_X[4] = (Control_Info->R_Leg_Info.Target.Phi              - Control_Info->R_Leg_Info.Measure.Phi);
	Control_Info->R_Leg_Info.LQR_X[5] = (Control_Info->R_Leg_Info.Target.Phi_dot          - Control_Info->R_Leg_Info.Measure.Phi_dot);




}


/*函数功能
1.​计算虚拟腿的推力（F）和扭矩（Tp）​​：
读关节电机的扭矩数据，结合机器人的几何参数（如杆长、角度等），计算出作用在虚拟腿上的顶力（F）和扭矩（Tp）。
分别计算左腿和右腿的F和Tp。

2.离地检测
​在平衡状态下计算支撑力（FN）和设置支撑标志​：
当底盘处于平衡状态（CHASSIS_BALANCE）时，
根据计算出的F和Tp，结合虚拟腿的倾斜角度（Theta）和长度（L0），
计算支撑力（FN）。
如果支撑力FN小于50N，则设置支撑标志（Flag）为1（表示腿处于支撑状态），否则为0。
非平衡状态下，默认躺在地上
支撑标志被重置为0，并给支撑力FN一个默认值（100N）。

逆运动学计算
求解由F和Tp到T1和T2的矩阵的逆矩阵
得到从T1和T2到F和Tp的映射
F= K(T2−T1​)/A,Tp =T1+T2

参数说明
A= (a*t*sinM)/S
​，其中：

M= 
θ_1−θ_2

t=acosϕ+S

a=AD=0.098m

b=DC=0.1176m

K= 98/215为给定比值

逆矩阵存在的条件是 A不为0且 K不为0，这通常当 θ_1不等于θ_2
​
 时成立。
*/

static void VMC_Measure_F_Tp_Calculate(Control_Info_Typedef *Control_Info){
//偏腿
	//左腿 
		//F    //公式：F=K(T2-T1)/A            //(T_左小腿 - T_左大腿) ps:公式相同，但左右腿在单独分析时，大腿的朝向不同，故需单独考虑，具体见整体模型简图
		Control_Info->L_Leg_Info.Measure.F  = -(Control_Info->L_Leg_Info.Biased.K * (Control_Info->L_Leg_Info.Biased.T_Calf - Control_Info->L_Leg_Info.Biased.T_Thigh))/Control_Info->L_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
	//修改：改为负号
		Control_Info->L_Leg_Info.Measure.Tp = -(Control_Info->L_Leg_Info.Biased.T_Thigh + Control_Info->L_Leg_Info.Biased.T_Calf);

	//右腿
		//F   //公式：F=K(T2-T1)/A  = （T2-T1）/(A/K)           //(T_右大腿 - T_右小腿)
		Control_Info->R_Leg_Info.Measure.F  = -(Control_Info->R_Leg_Info.Biased.K * (Control_Info->R_Leg_Info.Biased.T_Thigh - Control_Info->R_Leg_Info.Biased.T_Calf))/Control_Info->R_Leg_Info.A;
		//Tp  //公式：Tp=T1+T2
	//2025.11.18--修改，Tp测量值改为负
		Control_Info->R_Leg_Info.Measure.Tp = -(Control_Info->R_Leg_Info.Biased.T_Thigh + Control_Info->R_Leg_Info.Biased.T_Calf);
  											

//离地检测灯

	//当车平衡时
	if(Control_Info->Chassis_Situation == CHASSIS_BALANCE){
	//扭矩不变
	//左腿
	Control_Info->L_Leg_Info.Measure.Tp = Control_Info->L_Leg_Info.Measure.Tp;

	//求支持力
    Control_Info->L_Leg_Info.Support.FN =      Control_Info->L_Leg_Info.Measure.F  * arm_cos_f32(Control_Info->L_Leg_Info.Measure.Theta)
										                           +  (  (Control_Info->L_Leg_Info.Measure.Tp * arm_sin_f32(Control_Info->L_Leg_Info.Measure.Theta))
									                              /     Control_Info->L_Leg_Info.Sip_Leg_Length);
		
		
		
//右腿
    Control_Info->R_Leg_Info.Measure.Tp = -Control_Info->R_Leg_Info.Measure.Tp;//注意极性

																					//注意：2025.11.15.12：54(Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta)加了一个负号
	Control_Info->R_Leg_Info.Support.FN    =  Control_Info->R_Leg_Info.Measure.F * arm_cos_f32(Control_Info->R_Leg_Info.Measure.Theta)
										                        +(-(Control_Info->R_Leg_Info.Measure.Tp*arm_sin_f32(Control_Info->R_Leg_Info.Measure.Theta))/Control_Info->R_Leg_Info.Sip_Leg_Length);

		//检测标志
			//左腿
			Control_Info->L_Leg_Info.Support.Flag = (Control_Info->L_Leg_Info.Support.FN < 50.f);
			//右腿
			Control_Info->R_Leg_Info.Support.Flag = (Control_Info->R_Leg_Info.Support.FN < 50.f);
	//车不平衡时
	}else {
	      Control_Info->L_Leg_Info.Support.Flag = 0;//红灯
       Control_Info->R_Leg_Info.Support.Flag = 0;
	   Control_Info->L_Leg_Info.Support.FN   = 100.f;
	   Control_Info->R_Leg_Info.Support.FN   =100.f;


	}




}


//在VMC_Measure_F_Tp_Calculate得知现在车是否离地，之后根据支撑状态自适应调整控制策略，并生成最终的控制输出（力矩T和扭矩Tp
//不用改，并腿/偏腿通用
static void LQR_T_Tp_Calculate(Control_Info_Typedef *Control_Info){
//1. 支撑状态自适应调整	

	//当腿处于离地状态时（着地），​禁用水平方向控制​（轮子力矩T相关增益）
//    //​部分禁用垂直方向控制（关节扭矩Tp相关增益）
//	if(Control_Info->L_Leg_Info.Support.Flag == 1){

// 		// 清零左腿LQR增益矩阵的第一行（力矩T相关）
//		Control_Info->L_Leg_Info.LQR_K[0][0] = 0;
//		Control_Info->L_Leg_Info.LQR_K[0][1] = 0; 
//		Control_Info->L_Leg_Info.LQR_K[0][2] = 0;
//		Control_Info->L_Leg_Info.LQR_K[0][3] = 0; 
//		Control_Info->L_Leg_Info.LQR_K[0][4] = 0;
//		Control_Info->L_Leg_Info.LQR_K[0][5] = 0; 
//	

//	}
//	//右腿
//	if(Control_Info->R_Leg_Info.Support.Flag == 1){
//	
//		Control_Info->R_Leg_Info.LQR_K[0][0] = 0;
//		Control_Info->R_Leg_Info.LQR_K[0][1] = 0; 
//		Control_Info->R_Leg_Info.LQR_K[0][2] = 0;
//		Control_Info->R_Leg_Info.LQR_K[0][3] = 0; 
//		Control_Info->R_Leg_Info.LQR_K[0][4] = 0;
//		Control_Info->R_Leg_Info.LQR_K[0][5] = 0; 



//	}

//2. LQR控制输出计算---------------------------------------------------------
//输出 = K · X
//K = 增益矩阵
//X = 状态误差向量
//--------------------------------------------------------------------------
//
	Control_Info->L_Leg_Info.LQR_Output[0][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[0][0];
	Control_Info->L_Leg_Info.LQR_Output[0][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[0][1];
	Control_Info->L_Leg_Info.LQR_Output[0][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[0][2];
	Control_Info->L_Leg_Info.LQR_Output[0][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[0][3];
	Control_Info->L_Leg_Info.LQR_Output[0][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[0][4];
	Control_Info->L_Leg_Info.LQR_Output[0][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[0][5];

  Control_Info->L_Leg_Info.Moment.Balance_T =   Control_Info->L_Leg_Info.LQR_Output[0][0] + Control_Info->L_Leg_Info.LQR_Output[0][1] + Control_Info->L_Leg_Info.LQR_Output[0][2]
                                              + Control_Info->L_Leg_Info.LQR_Output[0][3] + Control_Info->L_Leg_Info.LQR_Output[0][4] + Control_Info->L_Leg_Info.LQR_Output[0][5];

	
	Control_Info->R_Leg_Info.LQR_Output[0][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[0][0];
	Control_Info->R_Leg_Info.LQR_Output[0][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[0][1];
	Control_Info->R_Leg_Info.LQR_Output[0][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[0][2];
	Control_Info->R_Leg_Info.LQR_Output[0][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[0][3];
	Control_Info->R_Leg_Info.LQR_Output[0][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[0][4];
	Control_Info->R_Leg_Info.LQR_Output[0][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[0][5];	
	

	Control_Info->R_Leg_Info.Moment.Balance_T =   Control_Info->R_Leg_Info.LQR_Output[0][0] + Control_Info->R_Leg_Info.LQR_Output[0][1] + Control_Info->R_Leg_Info.LQR_Output[0][2]
                                              + Control_Info->R_Leg_Info.LQR_Output[0][3] + Control_Info->R_Leg_Info.LQR_Output[0][4] + Control_Info->R_Leg_Info.LQR_Output[0][5];


	Control_Info->L_Leg_Info.LQR_Output[1][0] = Control_Info->L_Leg_Info.LQR_X[0]*Control_Info->L_Leg_Info.LQR_K[1][0];
	Control_Info->L_Leg_Info.LQR_Output[1][1] = Control_Info->L_Leg_Info.LQR_X[1]*Control_Info->L_Leg_Info.LQR_K[1][1];
	Control_Info->L_Leg_Info.LQR_Output[1][2] = Control_Info->L_Leg_Info.LQR_X[2]*Control_Info->L_Leg_Info.LQR_K[1][2];
	Control_Info->L_Leg_Info.LQR_Output[1][3] = Control_Info->L_Leg_Info.LQR_X[3]*Control_Info->L_Leg_Info.LQR_K[1][3];
	Control_Info->L_Leg_Info.LQR_Output[1][4] = Control_Info->L_Leg_Info.LQR_X[4]*Control_Info->L_Leg_Info.LQR_K[1][4];
	Control_Info->L_Leg_Info.LQR_Output[1][5] = Control_Info->L_Leg_Info.LQR_X[5]*Control_Info->L_Leg_Info.LQR_K[1][5];

    Control_Info->L_Leg_Info.Moment.Balance_Tp = -( Control_Info->L_Leg_Info.LQR_Output[1][0] + Control_Info->L_Leg_Info.LQR_Output[1][1] + Control_Info->L_Leg_Info.LQR_Output[1][2]
                                               + Control_Info->L_Leg_Info.LQR_Output[1][3] + Control_Info->L_Leg_Info.LQR_Output[1][4] + Control_Info->L_Leg_Info.LQR_Output[1][5]);
 
	
	Control_Info->R_Leg_Info.LQR_Output[1][0] = Control_Info->R_Leg_Info.LQR_X[0]*Control_Info->R_Leg_Info.LQR_K[1][0];
	Control_Info->R_Leg_Info.LQR_Output[1][1] = Control_Info->R_Leg_Info.LQR_X[1]*Control_Info->R_Leg_Info.LQR_K[1][1];
	Control_Info->R_Leg_Info.LQR_Output[1][2] = Control_Info->R_Leg_Info.LQR_X[2]*Control_Info->R_Leg_Info.LQR_K[1][2];
	Control_Info->R_Leg_Info.LQR_Output[1][3] = Control_Info->R_Leg_Info.LQR_X[3]*Control_Info->R_Leg_Info.LQR_K[1][3];
	Control_Info->R_Leg_Info.LQR_Output[1][4] = Control_Info->R_Leg_Info.LQR_X[4]*Control_Info->R_Leg_Info.LQR_K[1][4];
	Control_Info->R_Leg_Info.LQR_Output[1][5] = Control_Info->R_Leg_Info.LQR_X[5]*Control_Info->R_Leg_Info.LQR_K[1][5];	
	
	Control_Info->R_Leg_Info.Moment.Balance_Tp =   (Control_Info->R_Leg_Info.LQR_Output[1][0] + Control_Info->R_Leg_Info.LQR_Output[1][1] + Control_Info->R_Leg_Info.LQR_Output[1][2] + Control_Info->R_Leg_Info.LQR_Output[1][3] + Control_Info->R_Leg_Info.LQR_Output[1][4] + Control_Info->R_Leg_Info.LQR_Output[1][5]);
  


}	
	
//另类目标控制，只不过这里的控制是写死的
//各种功能是以打补丁的方式添加到对关节电机，驱动轮电机的控制中

static void Comprehensive_F_Calculate(Control_Info_Typedef *Control_Info){
// //测试(底盘专用)
// /*

// 主要是修改Yaw_Err的来源
// 将遥控器的摇杆值（范围：-660到660）代替DM_Yaw_Motor.Data.Position*/
 Control_Info->Yaw_Err = -(0.f - remote_ctrl.rc.ch[2] * RemoteToDegrees);

// 	//计算偏航角误差 Yaw_Err，通过将电机位置从弧度转换为角度后取负值
 	//Control_Info->Yaw_Err = 0.f - DM_Yaw_Motor.Data.Position * RadiansToDegrees ;
// 	//将偏航角误差限制在[-180~180]度
 	if (Control_Info->Yaw_Err >= 180.f) 		Control_Info->Yaw_Err -= 360.f;
 	else if (Control_Info->Yaw_Err <= -180.f)   Control_Info->Yaw_Err += 360.f;

// //yaw轴串级PID控制
// //上级：目标yaw轴角度偏差为0，输入Control_Info->Yaw_Err，输出：PID_Yaw[0].Output
 	PID_Calculate(&PID_Yaw[0], 0, Control_Info->Yaw_Err);
// //下级：目标yaw轴角速度为0，输入INS_Info.Yaw_Gyro，输出：PID_Yaw[1].Output
 	PID_Calculate(&PID_Yaw[1],PID_Yaw[0].Output,INS_Info.Yaw_Gyro);


  // 更新PID_Leg_Coordinate，设置目标值为左脚和右脚的Theta差值，反馈值为左脚和右脚的Theta差值
 
  // 防劈叉PID控制
  // 更新PID_Leg_Coordinate，设置目标值为左脚和右脚的Theta差值，反馈值为左脚和右脚的Theta差值
     PID_Calculate(&PID_Leg_Coordinate, 0, Control_Info->L_Leg_Info.Measure.Theta - Control_Info->R_Leg_Info.Measure.Theta);
// //影响简化腿摆角力矩Tp
// //将腿部坐标调节力矩分配给左右腿，方向相反以实现平衡
 	Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
 	Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp = -PID_Leg_Coordinate.Output;
// //横滚控制，在左右腿处于不同高度时，保持机体水平方向平衡	
// //基于滚转角度计算左右腿的滚动补偿力，增加稳定性。
 	Control_Info->L_Leg_Info.Moment.Roll_F = -(INS_Info.Roll_Angle + 0.4f) * 10.f;
 	Control_Info->R_Leg_Info.Moment.Roll_F =  (INS_Info.Roll_Angle + 0.4f) * 10.f;
	
// //转向控制	
// //将偏航转向力矩分别施加到左右腿上，方向相反实现转向。
 	Control_Info->L_Leg_Info.Moment.Turn_T =  PID_Yaw[1].Output;
 	Control_Info->R_Leg_Info.Moment.Turn_T = -PID_Yaw[1].Output;
// //腿长控制	
// 	//分别对左右腿长度进行 PID 控制，使腿长达到目标值。
 	//输入：目标腿长（sip_leg_length）和测量腿长（sip_leg_length）
 	//输出：PID_Leg_length_F[0].Output和PID_Leg_length_F[1].Output
   Control_Info->L_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[0],Control_Info->L_Leg_Info.Target_Sip_Leg_Length,Control_Info->L_Leg_Info.Sip_Leg_Length);
   Control_Info->R_Leg_Info.Moment.Leg_Length_F =  PID_Calculate(&PID_Leg_length_F[1],Control_Info->R_Leg_Info.Target_Sip_Leg_Length,Control_Info->R_Leg_Info.Sip_Leg_Length);

// //设置初始重力补偿值为 100。f
// Control_Info->R_Leg_Info.Gravity_Compensation = 40.f;
// Control_Info->L_Leg_Info.Gravity_Compensation = 40.f;
// //如果左腿处于支撑状态，则取消其滚动补偿，并提高重力补偿至 140。
// 	if(Control_Info->L_Leg_Info.Support.Flag == 1){
//	   
// 		Control_Info->L_Leg_Info.Moment.Roll_F = 0;
// 	  Control_Info->L_Leg_Info.Gravity_Compensation = 40.f;
// 	}
// 	//同理处理右腿支撑情况。
// 	if(Control_Info->R_Leg_Info.Support.Flag == 1){
//	
// 		Control_Info->R_Leg_Info.Moment.Roll_F = 0;
// 		Control_Info->R_Leg_Info.Gravity_Compensation = 40.f;

//	
// 	}

	
	
// 	//综合各部分力矩得到左右腿总的驱动力。
// 	//公式：F =  腿长补偿力（L_Leg_Info.Moment.Leg_Length_F）
// 	//         + 横滚补偿力（L_Leg_Info.Moment.Roll_F ）
// 	//         + 重力补偿（L_Leg_Info.Gravity_Compensation;）
// 	//分析：已知，重力的方向垂直地面向下，且Gravity_Compensation的值为正，故综合力的方向向下，
// 	//同重力方向
// 	Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F+ Control_Info->L_Leg_Info.Moment.Roll_F  +  Control_Info->L_Leg_Info.Gravity_Compensation;  
// 	Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F+ Control_Info->R_Leg_Info.Moment.Roll_F  +  Control_Info->R_Leg_Info.Gravity_Compensation;
	
  	//同重力方向
//  	Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F +  Control_Info->L_Leg_Info.Gravity_Compensation;  
//  	Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F +  Control_Info->R_Leg_Info.Gravity_Compensation;
//		
		
		 	Control_Info->L_Leg_Info.F = Control_Info->L_Leg_Info.Moment.Leg_Length_F +  Control_Info->L_Leg_Info.Gravity_Compensation;  
  	Control_Info->R_Leg_Info.F = Control_Info->R_Leg_Info.Moment.Leg_Length_F +  Control_Info->R_Leg_Info.Gravity_Compensation;
	
// // 	//驱动轮的转矩
// // 	//公式：T = 平衡力矩
 // 	//         +转向力矩
  	Control_Info->L_Leg_Info.T = Control_Info->L_Leg_Info.Moment.Balance_T  + Control_Info->L_Leg_Info.Moment.Turn_T ;
  	Control_Info->R_Leg_Info.T = Control_Info->R_Leg_Info.Moment.Balance_T  + Control_Info->R_Leg_Info.Moment.Turn_T ;

 // 	//简化腿的总转矩
 // 	//公式：Tp =  平衡转矩
 // 	//         + 双腿摆角补偿转矩
  	Control_Info->L_Leg_Info.Tp =   Control_Info->L_Leg_Info.Moment.Balance_Tp + Control_Info->L_Leg_Info.Moment.Leg_Coordinate_Tp;
  	Control_Info->R_Leg_Info.Tp =   Control_Info->R_Leg_Info.Moment.Balance_Tp + Control_Info->R_Leg_Info.Moment.Leg_Coordinate_Tp;
	
// 	if(Control_Info->L_Leg_Info.Support.Flag == 1){
//	   
// 			Control_Info->L_Leg_Info.T = 0;
//	
// 	}
//	
// 	if(Control_Info->R_Leg_Info.Support.Flag == 1){
//	
// 			Control_Info->R_Leg_Info.T = 0;
//	
//	
// 	}
//	
	
	
  	if(Control_Info->Chassis_Situation == CHASSIS_WEAK){
 
     Control_Info->L_Leg_Info.Tp = 0;
     Control_Info->R_Leg_Info.Tp = 0;
  	 Control_Info->L_Leg_Info.F = 0;
     Control_Info->R_Leg_Info.F = 0;	
  	 Control_Info->L_Leg_Info.T = 0;
     Control_Info->R_Leg_Info.T = 0;	
 
   }
/*
测试用：
1：简化模型参数到实际模型参数转换器
的测试
输入遥控器的值，转换到F和Tp


*/	



 	//    Control_Info->L_Leg_Info.Tp = remote_ctrl.rc.ch[2] *0.01f;
    //   Control_Info->R_Leg_Info.Tp = remote_ctrl.rc.ch[0] *0.01f;
 	//     Control_Info->L_Leg_Info.F  = remote_ctrl.rc.ch[3] *0.25f;
    //   Control_Info->R_Leg_Info.F  = remote_ctrl.rc.ch[1]  *0.25f;
//	  	 Control_Info->L_Leg_Info.T = 0;
//   Control_Info->R_Leg_Info.T = 0;	

}
//这个才是串并联腿实际的独特的函数转换框架，
//整个运控的逻辑是控制机器人运动时把它看成一个二阶独轮车
//目标层的数据都是基于二阶独轮车，只有这个函数才是把来自二阶独轮车的数据转化为真实的关节电机的力矩
//对应韭菜大佬的五连杆运动学解算和VMC-分析力学
//输入：上位平衡车模型的延杆伸缩力generate的力F和扭矩Tp
//输出：实际偏置并联结构的关节电机的力矩T1和T2

static void Joint_Tourgue_Calculate(Control_Info_Typedef *Control_Info){
//输入：简化力和简化转矩
//转换器：2X2的雅克比矩阵
//输出：真实结构中各电机的力矩
//原理：
//中间量：
	float Current_max;
float Tourgue_max;
	Tourgue_max = 35.f;
	Current_max =10000;

//偏腿
//左腿
//0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Calf = (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->L_Leg_Info.SendValue.T_Thigh = (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K + (Control_Info->L_Leg_Info.Tp/2.0f);
//右腿
//2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
	//2025年11.12修改：互换
Control_Info->R_Leg_Info.SendValue.T_Thigh = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);
//3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
Control_Info->R_Leg_Info.SendValue.T_Calf = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K + (Control_Info->R_Leg_Info.Tp/2.0f);



// ////偏腿
// ////左腿
// ////0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Calf =  (Control_Info->L_Leg_Info.Tp/2.0f);
// ////1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Thigh =  (Control_Info->L_Leg_Info.Tp/2.0f);
// ////右腿
// ////2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// //	//2025年11.12修改：互换
// Control_Info->R_Leg_Info.SendValue.T_Thigh = (Control_Info->R_Leg_Info.Tp/2.0f);
// ////3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->R_Leg_Info.SendValue.T_Calf = (Control_Info->R_Leg_Info.Tp/2.0f);

//测试F 转换


// ////偏腿
//// ////左腿
//// ////0号电机，左小腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Calf =   (-Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K;
//// ////1号电机，左大腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->L_Leg_Info.SendValue.T_Thigh =  (Control_Info->L_Leg_Info.A * Control_Info->L_Leg_Info.F)/Control_Info->L_Leg_Info.Biased.K;
//// ////2号电机，右大腿 对应T2 公式：T2 =（A/K）*F +(Tp/2.0f)
// //	//2025年11.12修改：互换
// Control_Info->R_Leg_Info.SendValue.T_Thigh = (-Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K;
//// ////3号电机，右小腿 对应T1 公式：T1 =（-A/K）*F + (Tp/2.0f)
// Control_Info->R_Leg_Info.SendValue.T_Calf = (Control_Info->R_Leg_Info.A * Control_Info->R_Leg_Info.F)/Control_Info->R_Leg_Info.Biased.K;




//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------											 
//以下部分不用改，串并联通用部分
//轮子电机的电流值
//	 Control_Info->L_Leg_Info.SendValue.Current = (int16_t)( Control_Info->L_Leg_Info.T * 1000.f);
//	 Control_Info->R_Leg_Info.SendValue.Current = (int16_t)( -Control_Info->R_Leg_Info.T * 1000.f);

	 Control_Info->L_Leg_Info.SendValue.Current = (int16_t)( Control_Info->L_Leg_Info.T * 1000.f);
	 Control_Info->R_Leg_Info.SendValue.Current = (int16_t)( -Control_Info->R_Leg_Info.T * 1000.f);

	 
	 VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.Current,-Current_max,Current_max);
	 VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.Current,-Current_max,Current_max); 
  
	//  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T1,-54.f,54.f);
	//  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T2,-54.f,54.f);  
    //  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T1,-54.f,54.f);
	//  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T2,-54.f,54.f);  			
																		
	  
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Calf ,-Tourgue_max,Tourgue_max);
	  VAL_LIMIT(Control_Info->L_Leg_Info.SendValue.T_Thigh,-Tourgue_max,Tourgue_max);  
    VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Thigh,-Tourgue_max,Tourgue_max);
	  VAL_LIMIT(Control_Info->R_Leg_Info.SendValue.T_Calf ,-Tourgue_max,Tourgue_max);  			
																				
}