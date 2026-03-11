#include "dummy_kinematic_v2.h"

/**
 * @brief 矩阵乘法
 * @param _m A的行数
 * @param _l A的列数 (也是B的行数)
 * @param _n B的列数
 */
static void MatMultiply(const float *_matrix1, const float *_matrix2, float *_matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

/**
 * @brief 旋转矩阵转欧拉角 (ZYX顺序: Roll->Pitch->Yaw)
 *        用于正解计算末端姿态
 */
static void RotMatToEulerAngle(const float *_rotationM, float *_eulerAngles)
{
    float A, B, C;

    // 检查 Gimbal Lock (万向锁) 情况
    if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f)
    {
        if (_rotationM[6] < 0)
        {
            A = 0.0f;
            B = (float)M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
        else
        {
            A = 0.0f;
            B = -(float)M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    }
    else
    {
        // 正常情况求解
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        float cb = cosf(B);
        // 防止除零
        if (fabsf(cb) < 0.0001f)
            cb = 0.0001f;
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }
    // 保持计算不变，修正返回
    _eulerAngles[0] = A; // Yaw (Z) ← A 算的是 yaw
    _eulerAngles[1] = B; // Pitch (Y)
    _eulerAngles[2] = C; // Roll (X) ← C 算的是 roll
}

/**
 * @brief 欧拉角转旋转矩阵
 *        用于逆解时将目标姿态转为矩阵
 */
static void EulerAngleToRotMat(const float *_eulerAngles, float *_rotationM)
{
    // 明确命名，避免混淆
    float cy = cosf(_eulerAngles[0]), sy = sinf(_eulerAngles[0]);  // Yaw   (Z)
    float cp = cosf(_eulerAngles[1]), sp = sinf(_eulerAngles[1]);  // Pitch (Y)
    float cr = cosf(_eulerAngles[2]), sr = sinf(_eulerAngles[2]);  // Roll  (X)

    // R = Rz * Ry * Rx
    _rotationM[0] = cy * cp;
    _rotationM[1] = cy * sp * sr - sy * cr;
    _rotationM[2] = cy * sp * cr + sy * sr;
    
    _rotationM[3] = sy * cp;
    _rotationM[4] = sy * sp * sr + cy * cr;
    _rotationM[5] = sy * sp * cr - cy * sr;
    
    _rotationM[6] = -sp;
    _rotationM[7] = cp * sr;
    _rotationM[8] = cp * cr;
}


// -------------------------------------------------------------------------
//                            核心接口实现
// -------------------------------------------------------------------------

/**
 * @brief 初始化运动学句柄
 */
void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config)
{
    // 1. 深拷贝配置参数
    memcpy(&handle->config, config, sizeof(ArmConfig_t));
    
    // 为了写代码方便，定义局部指针指向配置
    const ArmConfig_t *cfg = &handle->config;

    // 2. 初始化标准 DH 参数矩阵 [theta, d, a, alpha]
    // 使用配置中的参数填充
    // float tmp_DH[6][4] = {
    //     {0.0f,                       cfg->L_BASE,    cfg->D_BASE,  -(float)M_PI_2},
    //     {-75.0f * DEG_TO_RAD_CONST,  0.0f,           cfg->L_ARM,   0.0f},
    //     {-90.0f * DEG_TO_RAD_CONST,  cfg->D_ELBOW,   0.0f,         (float)M_PI_2},
    //     {0.0f,                       cfg->L_FOREARM, 0.0f,         -(float)M_PI_2},
    //     {0.0f,                       0.0f,           0.0f,         (float)M_PI_2},
    //     {0.0f,                       cfg->L_WRIST,   0.0f,         0.0f}
    // };
    float tmp_DH[6][4] = {
        {0.0f,                       cfg->L_BASE,    cfg->D_BASE,  -(float)M_PI_2},
        {0.0f,                       0.0f,           cfg->L_ARM,   0.0f},
        {0.0f,                       cfg->D_ELBOW,   0.0f,         (float)M_PI_2},
        {0.0f,                       cfg->L_FOREARM, 0.0f,         -(float)M_PI_2},
        {0.0f,                       0.0f,           0.0f,         (float)M_PI_2},
        {0.0f,                       cfg->L_WRIST,   0.0f,         0.0f}
    };    
    memcpy(handle->DH_matrix, tmp_DH, sizeof(tmp_DH));
    // 3. 预计算向量
    float tmp_L1[3] = {cfg->D_BASE, -cfg->L_BASE, 0.0f};
    memcpy(handle->L1_base, tmp_L1, sizeof(tmp_L1));
    
    float tmp_L2[3] = {cfg->L_ARM, 0.0f, 0.0f};
    memcpy(handle->L2_arm, tmp_L2, sizeof(tmp_L2));
    
    float tmp_L3[3] = {-cfg->D_ELBOW, 0.0f, cfg->L_FOREARM};
    memcpy(handle->L3_elbow, tmp_L3, sizeof(tmp_L3));

    float tmp_L6[3] = {0.0f, 0.0f, cfg->L_WRIST};
    memcpy(handle->L6_wrist, tmp_L6, sizeof(tmp_L6));

    // 4. 预计算平方项
    handle->l_se_2 = cfg->L_ARM * cfg->L_ARM;
    handle->l_se = cfg->L_ARM;
    handle->l_ew_2 = cfg->L_FOREARM * cfg->L_FOREARM + cfg->D_ELBOW * cfg->D_ELBOW;

    // 5. 立即计算完整几何参数, 避免运行时 Lazy Init
    // 如果 l_ew (Effective Wrist Length) 为 0 (未初始化状态), 会导致除0错误
    // 肘部到腕部的直线距离 (小臂有效长度)
    // l_ew = sqrt(forearm^2 + elbow_offset^2)
    if (handle->l_ew_2 > 0.000001f)
        handle->l_ew = sqrtf(handle->l_ew_2);
    else
        handle->l_ew = 0.0f;
    // 计算肘部固有偏移角度 (Atan Elbow)
    // 这是由于 D_ELBOW 造成的结构性偏移角
    // 假设 D_ELBOW 相对于 L_FOREARM 是垂直关系
    // atan_e = atan2(D_ELBOW, L_FOREARM)
    // 注意: 具体正负号取决于结构定义, 这里假设 D_ELBOW 使小臂"翘起"
    handle->atan_e = atan2f(cfg->D_ELBOW, cfg->L_FOREARM);
}

// --- FK 核心算法 ---

/**
 * @brief 正运动学解算 (Joints -> Pose)
 *        使用改进的标准 DH 参数法，与 Init 中的 DH 表对其
 */
bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle, const Joint6D_t *inputJoints, Pose6D_t *outputPose)
{
    // 标准 DH 转换矩阵 T_i-1,i
    // [ ct   -st*ca   st*sa   a*ct ]
    // [ st    ct*ca  -ct*sa   a*st ]
    // [ 0     sa      ca      d    ]
    // [ 0     0       0       1    ]

    float T_total[16]; // T0_i
    float T_next[16];  // T_i-1_i
    float T_tmp[16];   // Temp

    // 初始化为单位矩阵
    memset(T_total, 0, sizeof(T_total));
    T_total[0] = 1.0f; T_total[5] = 1.0f; T_total[10] = 1.0f; T_total[15] = 1.0f;

    for (int i = 0; i < 6; i++)
    {
        // 1. 获取 DH 参数 (来自 Kinematic_Init 初始化)
        float theta_offset = handle->DH_matrix[i][0];
        float d            = handle->DH_matrix[i][1];
        float a            = handle->DH_matrix[i][2];
        float alpha        = handle->DH_matrix[i][3];

        // 2. 实际关节角 (输入角度转弧度 + 偏移)
        float theta = inputJoints->a[i] * DEG_TO_RAD_CONST + theta_offset;

        // 3. 计算当前变换矩阵 T (使用标准DH公式)
        float ct = cosf(theta);
        float st = sinf(theta);
        float ca = cosf(alpha);
        float sa = sinf(alpha);

        T_next[0] = ct;   T_next[1] = -st*ca;  T_next[2] = st*sa;   T_next[3] = a*ct;
        T_next[4] = st;   T_next[5] = ct*ca;   T_next[6] = -ct*sa;  T_next[7] = a*st;
        T_next[8] = 0;    T_next[9] = sa;      T_next[10] = ca;     T_next[11] = d;
        T_next[12] = 0;   T_next[13] = 0;      T_next[14] = 0;      T_next[15] = 1;

        // 4. 累乘: T_total = T_total * T_next
        MatMultiply(T_total, T_next, T_tmp, 4, 4, 4);
        memcpy(T_total, T_tmp, sizeof(float)*16);
    }

    // 提取旋转矩阵并转欧拉角
    float R06[9];
    R06[0] = T_total[0]; R06[1] = T_total[1]; R06[2] = T_total[2];
    R06[3] = T_total[4]; R06[4] = T_total[5]; R06[5] = T_total[6];
    R06[6] = T_total[8]; R06[7] = T_total[9]; R06[8] = T_total[10];

    // 回填旋转矩阵到 outputPose
    memcpy(outputPose->R, R06, 9 * sizeof(float));
    outputPose->hasR = true;

    // 计算欧拉角 (Euler ZYX -> Roll Pitch Yaw)
    float euler[3];
    RotMatToEulerAngle(R06, euler);
    
    // 提取位置 (最后一列的 X, Y, Z)
    // T_total = [ R  P ]
    //           [ 0  1 ]
    outputPose->X = T_total[3]; // X
    outputPose->Y = T_total[7]; // Y
    outputPose->Z = T_total[11]; // Z
    // 转回角度制,RotMatToEulerAngle 返回: euler[0]=yaw, [1]=pitch, [2]=roll
    outputPose->A = euler[0] * RAD_TO_DEG_CONST;  // Yaw   (Z-axis rotation)
    outputPose->B = euler[1] * RAD_TO_DEG_CONST;  // Pitch (Y-axis rotation)  
    outputPose->C = euler[2] * RAD_TO_DEG_CONST;  // Roll  (X-axis rotation)

    return true;
}



// --- IK 核心算法 ---

/**
 * @brief 逆运动学解算 (Pose -> Joints)
 *        使用几何法 (Geometric Approach) + 腕部解耦 (Pieper's Solution)
 *        针对 6-DOF 且满足以下条件的机械臂：
 *        1. 后三轴 (J4, J5, J6) 轴线交于一点 (Wrist Center)
 *        2. J2, J3 平行
 *        3. J1 垂直于 J2
 */
bool Kinematic_SolveIK(const DOF6Kinematic_Handle_t *handle,
                       const Pose6D_t *inputPose,
                       const Joint6D_t *lastJoints,
                       IKSolves_t *outputSolves)
{
    // 0. 几何常数提取 (减少重复访问指针)
    float l1 = handle->config.D_BASE;      // J1 水平偏移
    float l2 = handle->config.L_ARM;       // 大臂长
    // l3, d4 未使用，因为我们直接用 l_ew (预计算好的有效长度)
    // float l3 = handle->config.L_FOREARM;   
    // float d4 = handle->config.D_ELBOW;     
    float l4 = handle->config.L_WRIST;     // 手腕长 (J6 法兰距离)
    
    // 腕部等效长度 (Elbow to Wrist 连线长度)
    // l_ew = sqrt(l3^2 + d4^2)
    float l_ew = handle->l_ew;
    // 肘部固有偏移角 atan(d4/l3)
    float atan_e = handle->atan_e; 

    // 1. 准备目标位姿矩阵 (T06)
    float R06[9]; // 旋转矩阵
    float P[3];   // 目标位置

    P[0] = inputPose->X;
    P[1] = inputPose->Y;
    P[2] = inputPose->Z;

    if (inputPose->hasR) 
    {
        memcpy(R06, inputPose->R, 9 * sizeof(float));
    } 
    else 
    {
        // 默认根据欧拉角(Z-Y-X)构建旋转矩阵
        float pitch = inputPose->B * DEG_TO_RAD_CONST;
        float roll  = inputPose->C * DEG_TO_RAD_CONST;
        float yaw   = inputPose->A * DEG_TO_RAD_CONST;
        
        float sp = sinf(pitch);
        float cp = cosf(pitch);
        float sr = sinf(roll);
        float cr = cosf(roll);
        float sy = sinf(yaw);
        float cy = cosf(yaw);

        R06[0] = cy * cp;
        R06[1] = cy * sp * sr - sy * cr;
        R06[2] = cy * sp * cr + sy * sr;

        R06[3] = sy * cp;
        R06[4] = sy * sp * sr + cy * cr;
        R06[5] = sy * sp * cr - cy * sr;

        R06[6] = -sp;
        R06[7] = cp * sr;
        R06[8] = cp * cr;
    }

    // 2. 计算腕部中心点 (Wrist Center, P_w)
    // P_w = P_end - l4 * (R06 * [0,0,1]^T)
    // 也就是 P_w = P_end - l4 * Z_axis_of_R06
    float P_w[3];
    P_w[0] = P[0] - l4 * R06[2]; // R06 第三列即为 Z 轴向量
    P_w[1] = P[1] - l4 * R06[5];
    P_w[2] = P[2] - l4 * R06[8];

    // 初始化解的有效性标记
    memset(outputSolves->solFlag, 0, sizeof(outputSolves->solFlag));

    // ==========================================================
    // 求解关节 1 (theta1) - 两个解 (Front / Back)
    // ==========================================================
    // 投影到 XY 平面
    // P_w_x = c1 * x + s1 * y (在 J1 坐标系下的长度)
    // 需要满足: P_w_x^2 + P_w_y^2 >= d_base^2 (如果有 d_base 偏移)
    // 但通常 J1 轴线穿过原点，l1 是 J2 相对 J1 的水平偏移？
    // 根据你的 DH 表：
    // J1: alpha = -90, a = D_BASE. 意味着 J2 轴线偏离 J1 轴线 D_BASE 距离
    // 这种情况 J1 的解法是：
    // rho^2 = Pwx^2 + Pwy^2
    // 如果 rho < D_BASE，无解。
    // theta1 = atan2(Py, Px) - atan2(D_BASE, +/- sqrt(rho^2 - D_BASE^2))
    
    // 如果 D_BASE 是沿 X 轴的偏移 (a 参数)，则：
    float rho2 = P_w[0]*P_w[0] + P_w[1]*P_w[1];
    float rho  = sqrtf(rho2);
    
    // 如果 D_BASE 定义为 DH 的 d 参数(沿z轴)，则不需要 acos/asin 修正，直接 atan2
    // 查看你的 Init DH： J1: d=L_Base, a=D_Base.
    
    // 3. 求解 J1
    float theta1_1 = 0.0f;
    float theta1_2 = 0.0f;
    float phi_offset = 0.0f;
    
    // [检查奇异性] : 腕点在 Z 轴附近
    if (rho < 0.0001f)
    {
        // 奇异，保持上一时刻角度
        // 这种情况下任意角度都行，取上一时刻最稳
        float current_j1 = lastJoints->a[0] * DEG_TO_RAD_CONST;
        outputSolves->config[0].a[0] = current_j1;
        outputSolves->config[4].a[0] = current_j1;
        // 标记为潜在问题，但继续计算
        theta1_1 = current_j1;
        theta1_2 = current_j1;
    } 
    else 
    {
        // [几何解法] 
        float alpha = atan2f(P_w[1], P_w[0]);
        // D_BASE 的影响已经通过几何中心定义处理，这里主要处理 D_ELBOW (J3侧向偏移) 的影响
        float d_elbow_offset = handle->config.D_ELBOW;

        if (rho > fabsf(d_elbow_offset))
        {
            // 计算由 D_ELBOW 引起的角度偏差
            // asin(offset / horizontal_dist)
            phi_offset = asinf(d_elbow_offset / rho);
            
            // 两个可能的解：正向伸手 和 反向伸手
            // 减去 phi_offset 是为了补偿侧向偏移
            // 此时的手臂平面投影长度不再是 rho，而是 sqrt(rho^2 - e^2)
            // 这一点会在后面计算 J2/J3 时用到 (rx)
        }
        else
        {
            // 目标点在“圆柱盲区”内，无法到达
            // 这种情况下只能尽力而为，保持 alpha
            phi_offset = 0.0f;
        }

        theta1_1 = alpha - phi_offset;
        theta1_2 = alpha + (float)M_PI + phi_offset; // 背面解

        // 将角度标准化到 -PI ~ PI
        while (theta1_1 > (float)M_PI) theta1_1 -= 2.0f * (float)M_PI;
        while (theta1_1 < -(float)M_PI) theta1_1 += 2.0f * (float)M_PI;
        
        while (theta1_2 > (float)M_PI) theta1_2 -= 2.0f * (float)M_PI;
        while (theta1_2 < -(float)M_PI) theta1_2 += 2.0f * (float)M_PI;
        
        // 填入两组大类 (0-3 为 J1解1, 4-7 为 J1解2)
        for(int i=0; i<4; i++) outputSolves->config[i].a[0] = theta1_1;
        for(int i=4; i<8; i++) outputSolves->config[i].a[0] = theta1_2;
    }
    
    // 针对 Theta1 的两种情况遍历
    for (int ind_arm = 0; ind_arm < 2; ind_arm++)
    {
        // [新增] 显式计算当前循环使用的 theta1 及其三角函数
        // 用于精确计算 J2 坐标系下的投影
        float th1_curr = outputSolves->config[ind_arm * 4].a[0];
        
        // ==========================================================
        // 求解关节 2, 3 (theta2, theta3) - 平面几何
        // ==========================================================
        // 此时我们将 3D 问题转化为了 2D 平面问题
        
        // [关键修正]
        // 当存在 D_ELBOW 时，末端并不在 J1 的径向平面上，而是有一个偏移。
        
        // 如果 d_elbow_offset 为 0，sqrtf(rho2) = rho
        // 注意这里的 d_elbow_offset 是 handle->config.D_ELBOW，需要在 loop 外获取或者直接使用 handle->
        float d_elbow = handle->config.D_ELBOW;
        float rx_1 = sqrtf(fmaxf(0.0f, rho2 - d_elbow * d_elbow)); 
        // 还需要减去 J1 的水平偏移 L1 (即 A1/D_BASE，视 DH 定义而定)
        rx_1 -= handle->config.D_BASE;

        // rz 是 Z 方向的高度差
        // J2 的原点高度是 L_BASE
        float rz_1 = P_w[2] - handle->config.L_BASE;

        float rx_2 = -sqrtf(fmaxf(0.0f, rho2 - d_elbow * d_elbow)) - handle->config.D_BASE;
        float rz_2 = rz_1;

        float rx, rz;
        if (ind_arm == 0) {
            rx = rx_1; 
            rz = rz_1;
        } else {
            rx = rx_2; 
            rz = rz_2; 
        }

        // 重算 c2 (Target distance squared)
        float dist_sq = rx*rx + rz*rz;
        float dist = sqrtf(dist_sq);

        float a1 = l2;   // 大臂
        float a2 = l_ew; // 小臂等效长度
        
        // 计算 J3 关节位置的几何两解 (Elbow Up / Elbow Down)
        // 利用余弦定理
        float cos_phi = (l2*l2 + l_ew*l_ew - dist_sq) / (2.0f * l2 * l_ew); 
        
        // [优化] 增加浮点数容差处理
        if (cos_phi > 1.0f) cos_phi = 1.0f;
        if (cos_phi < -1.0f) cos_phi = -1.0f;
        
        float phi = acosf(cos_phi); // 三角形内角 (0 ~ PI)
        
        // J3 的两个解: Elbow Up / Elbow Down
        // 根据 DH 定义修正: 
        // 你的 init DH 中 J3 的 offset 是 90 度?
        // 我们先算几何角，最后转 DH
        
        // Elbow Up
        float theta3_1 = (float)M_PI - phi - atan_e; 
        // Elbow Down
        float theta3_2 = -(float)M_PI + phi - atan_e;

        // 求解 theta2
        // theta2 = atan2(rz, rx) - (三角形底角 gamma)
        // cos_gamma = (l2^2 + s^2 - l_ew * l_ew) / (2 * l2 * s)
        float cos_gamma = (l2*l2 + dist_sq - l_ew*l_ew) / (2.0f * l2 * dist); 
        
        if (cos_gamma > 1.0f) cos_gamma = 1.0f;
        if (cos_gamma < -1.0f) cos_gamma = -1.0f;

        float gamma = acosf(cos_gamma);
        float beta = atan2f(rz, rx);
        
        float theta2_1 = beta - gamma; // 对应 theta3_1 (Elbow Up)
        float theta2_2 = beta + gamma; // 对应 theta3_2 (Elbow Down)
        
        // ==========================================================
        // [调试诊断] 
        // 这一步非常关键。上面的 theta2_1, theta3_1 是纯几何角度（水平为0，拉直为0）
        // 下面要根据 DH 表里的 Offset 转换成电机角度。
        // 如果这里出错，会导致解完全对不上。
        // ==========================================================
        
        float off2 = handle->DH_matrix[1][0];
        float off3 = handle->DH_matrix[2][0];
        
        // 临时变量保存几何解，用于 Debug (不做 Offset 修正)
        float th2_geom_1 = theta2_1;
        float th3_geom_1 = theta3_1;

        // Joint = Geom - Offset
        theta2_1 -= off2;
        theta2_2 -= off2;
        theta3_1 -= off3;
        theta3_2 -= off3;
        
        // 填入解
        // Group 1: J1_1/2, J2_1, J3_1 (Up)
        // Group 2: J1_1/2, J2_2, J3_2 (Down)
        
        // 存入 solve struct
        int base_idx = ind_arm * 4;
        
        // Elbow Up
        outputSolves->config[base_idx + 0].a[1] = theta2_1;
        outputSolves->config[base_idx + 0].a[2] = theta3_1;
        
        // [Hack for Debug] 把纯几何解存入第 4/5 个轴的位置，方便在 Watch 窗口查看
        // 如果您看到 a[3] 和 a[4] 的值接近 75 和 90 (弧度制)，说明几何算对了，是 Offset 问题。
        outputSolves->config[base_idx + 0].a[3] = th2_geom_1;
        outputSolves->config[base_idx + 0].a[4] = th3_geom_1;
        
        outputSolves->config[base_idx + 1].a[1] = theta2_1;
        outputSolves->config[base_idx + 1].a[2] = theta3_1;
        
        // Elbow Down
        outputSolves->config[base_idx + 2].a[1] = theta2_2;
        outputSolves->config[base_idx + 2].a[2] = theta3_2;
        outputSolves->config[base_idx + 3].a[1] = theta2_2;
        outputSolves->config[base_idx + 3].a[2] = theta3_2;
        
        // ==========================================================
        // 求解关节 4, 5, 6 (Spherical Wrist)
        // ==========================================================
        // R_wrist = (R0_1 * R1_2 * R2_3)^T * R0_6
        // R0_3 = R0_1 * R1_2 * R2_3
        
        for (int i_elbow = 0; i_elbow < 2; i_elbow++) 
        {
            int current_idx_base = base_idx + i_elbow * 2;
            int idx1 = current_idx_base;     // Flip
            int idx2 = current_idx_base + 1; // No Flip
            
            float th1_real = outputSolves->config[idx1].a[0];
            float th2_real = outputSolves->config[idx1].a[1];
            float th3_real = outputSolves->config[idx1].a[2];
            
            // 计算 R0_3
            // 这里为了准确，必须代入 DH 参数算旋转矩阵
            // T01 * T12 * T23 的旋转部分
            // 简化计算：R03 = RotZ(t1)*RotX(a1)*RotZ(t2)*RotX(a2)*RotZ(t3)
            // 根据你的 DH 表:
            // J1: alpha=-90
            // J2: alpha=0
            // J3: alpha=90
            
            // 下面构造旋转矩阵需要严格按照 Init 里的 DH 表
            // 建议：直接算出来 R03
            
            // 1. Calculate R03 matrices
            // [关键] 正向运动学推导中，R矩阵构建必须包含 Offset
            // R = RotZ(theta + offset) * ...
            // 因为 R01, R12, R23 是用的真实 theta (算法解 + Offset) 吗？
            // 不，这里的 th1_real, th2_real 是 outputSolves 里的值 (即 Joint Angle)
            // 而 DH 变换中的角度应该是 (Joint Angle + Offset)
            // 所以下面的 sinf/cosf 参数里必须加上 handle->DH_matrix[i][0]
            
            float c1_real = cosf(th1_real + handle->DH_matrix[0][0]); 
            float s1_real = sinf(th1_real + handle->DH_matrix[0][0]);
            float c2_real = cosf(th2_real + handle->DH_matrix[1][0]); 
            float s2_real = sinf(th2_real + handle->DH_matrix[1][0]); 
            float c3_real = cosf(th3_real + handle->DH_matrix[2][0]);
            float s3_real = sinf(th3_real + handle->DH_matrix[2][0]);
            
            // DH alpha: -90, 0, 90
            // T1(R) = [c1 -s1*0  s1*-1] = [c1  0 -s1]
            //         [s1  c1*0 -c1*-1]   [s1  0  c1]
            //         [0   -1    0    ]   [0  -1   0]
            
            float R01[9] = {c1_real, 0, -s1_real,  s1_real, 0, c1_real,  0, -1, 0}; // alpha1 = -90
            float R12[9] = {c2_real, -s2_real, 0,  s2_real, c2_real, 0,  0, 0, 1};  // alpha2 = 0
            float R23[9] = {c3_real, 0, s3_real,   s3_real, 0, -c3_real, 0, 1, 0};  // alpha3 = 90

            float R02[9], R03[9];
            MatMultiply(R01, R12, R02, 3, 3, 3);
            MatMultiply(R02, R23, R03, 3, 3, 3);
            
            // R36 = R03^T * R06
            float R03_T[9];
            // Transpose
            R03_T[0]=R03[0]; R03_T[1]=R03[3]; R03_T[2]=R03[6];
            R03_T[3]=R03[1]; R03_T[4]=R03[4]; R03_T[5]=R03[7];
            R03_T[6]=R03[2]; R03_T[7]=R03[5]; R03_T[8]=R03[8];
            
            float R36[9];
            MatMultiply(R03_T, R06, R36, 3, 3, 3);
            
            // 从 R36 中提取 Euler ZYZ (或者 ZYX? 看后三轴构造)
            // 你的 DH 后三轴:
            // J4: alpha = -90 (Rot X -90)
            // J5: alpha = 90  (Rot X 90)
            // J6: alpha = 0
            // 标准 Spherical Wrist 通常是 Z-Y-Z 类型的欧拉角解
            
            // R36 = RotZ(t4) * RotX(-90) * RotZ(t5) * RotX(90) * RotZ(t6)
            //     = [ c4c5c6-s4s6, -c4c5s6-s4c6, c4s5]
            //       [ s4c5c6+c4s6, -s4c5s6+c4c6, s4s5]
            //       [ -s5c6,       s5s6,         c5  ]
            // 观察 R33 (第九个元素) = c5
            
            float theta5 = acosf(R36[8]); // 0 ~ PI
            
            // 解 1: theta5 > 0 (No Flip) -> idx2
            // 解 2: theta5 < 0 (Flip)    -> idx1
            
            // Case 1: Positive sin(t5)
            outputSolves->config[idx2].a[4] = theta5;
            if (fabsf(sinf(theta5)) > 0.001f) 
            {
                outputSolves->config[idx2].a[3] = atan2f(R36[5], R36[2]); // atan2(s4s5, c4s5) = atan2(R23, R13)
                outputSolves->config[idx2].a[5] = atan2f(R36[7], -R36[6]); // atan2(s5s6, -s5c6)
            } 
            else 
            {
                // Singularity (J5 = 0), J4 and J6 align
                // [优化] 更加稳健的奇异点处理
                float t4_fixed = lastJoints->a[3] * DEG_TO_RAD_CONST;
                // 当 t5=0, R36[0] = c(t4+t6), R36[1] = -s(t4+t6) (假设是 RzRyRz)
                // 需根据实际旋转矩阵构造推导。
                // 假设标准球腕: R = Rz(4)*Ry(5)*Rz(6)
                // t5=0 => R = Rz(t4+t6) = [ c(4+6) -s(4+6) 0 ]
                //                         [ s(4+6)  c(4+6) 0 ]
                //                         [ 0       0      1 ]
                // atan2(R[1][0], R[0][0]) = t4+t6 
                // 注意 R36 是行主序平铺: 
                // index: 0 1 2
                //        3 4 5
                //        6 7 8
                // R36[3] = s(4+6), R36[0] = c(4+6)
                float sum_angle = atan2f(R36[3], R36[0]); 
                outputSolves->config[idx2].a[3] = t4_fixed;
                outputSolves->config[idx2].a[5] = sum_angle - t4_fixed;
                // 标记为奇异解
                outputSolves->solFlag[idx2][0] = 2; 
            }
            
            // Case 2: Negative sin(t5) -> theta5' = -theta5
            outputSolves->config[idx1].a[4] = -theta5;
             if (fabsf(sinf(theta5)) > 0.001f)
            {
                outputSolves->config[idx1].a[3] = atan2f(-R36[5], -R36[2]);
                outputSolves->config[idx1].a[5] = atan2f(-R36[7], R36[6]);
            } 
            else 
            {
                 // Singularity same handling
                float t4_fixed = lastJoints->a[3] * DEG_TO_RAD_CONST;
                float sum_angle = atan2f(R36[3], R36[0]); 
                outputSolves->config[idx1].a[3] = t4_fixed;
                outputSolves->config[idx1].a[5] = sum_angle - t4_fixed;
                outputSolves->solFlag[idx1][0] = 2;
            }
            
            // DH Offsets Correction for J4, J5, J6?
            // 你的 DH J4, J5, J6 都是 0 offset 吗？
            // 如果是，直接用。
        
        } // end loop wrist
    } // end loop j1

    // 4. 统一单位转换 (Rad -> Deg) 和范围归约
    for (int k = 0; k < 8; k++) 
    {
        // 如果前面标记过 solFlag = -1 (无解)，可以做进一步处理
        
        for (int j = 0; j < 6; j++) 
        {
            float ang = outputSolves->config[k].a[j];
            
            // [优化] 使用 fmod 替代 while 循环 进行归一化
            // 映射到 [-PI, PI]
            ang = fmodf(ang + (float)M_PI, 2.0f * (float)M_PI); 
            if (ang < 0) ang += 2.0f * (float)M_PI;
                ang -= (float)M_PI;
             
             // Convert to Degrees
            outputSolves->config[k].a[j] = ang * RAD_TO_DEG_CONST;
        }
        // 简单标记所有计算出的解有效 (你可以根据关节限位进一步筛选)
        // 如果之前没有被标记为奇异(2) 或 无效(-1)，则标记为正常(1)
        if (outputSolves->solFlag[k][0] == 0)
            outputSolves->solFlag[k][0] = 1;
    }

    return true;
}










/**
 * @brief 获取所有电机角度并填入 Joint_Angle_State_t 结构体
 * @param[out] joint_angle 输出的目标结构体指针
 */
void Joint_Motor_Get_All_Angles_And_State(Joint_Angle_State_t *joint_angle)
{
    // 安全检查
    if (joint_angle == NULL) return;

    // 遍历 Axis 1~6 (电机ID通常是1-6)
    // 注意: Joint_Angle_State_t 数组均从 0 开始 (angle[0]对应轴1)
    for(uint8_t i=0; i<6; i++) 
    {
        float angle = Joint_Motor_Get_Angle(i+1);
        uint8_t state = Joint_Motor_Get_State(i+1);
        joint_angle->angle[i] = angle;
        joint_angle->state[i] = state;
    }
}