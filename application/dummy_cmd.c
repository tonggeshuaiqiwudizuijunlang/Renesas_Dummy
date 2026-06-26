#include "dummy_cmd.h"
#include "message_center.h"
#include "flysky.h"
#include "bluetooth.h"
#include "serial_debug.h"
#include "vision.h"
#include "dummy_motormatic.h"



FS_ctrl_t *fs_data;
Publisher_t *dummy_cmd_pub;           // 控制消息发布者
Subscriber_t *dummy_feed_sub;         // 反馈信息订阅者
Dummy_Ctrl_Cmd_s dummy_cmd_send;      // 控制命令缓存
Dummy_Upload_Data_s dummy_fetch_data; // 反馈数据缓存

Transmit_Data_s vision_tx_data;  // 发送给上位机的数据缓存
Received_Data_s *vision_rx_data; // 从上位机接收的数据缓存
DOF6Kinematic_Handle_t *cmd_kinematic_handle;

SerialDebug_Instance_s *serial_debug_instance;

RX_BT_Data_s *bt_rx_data;
TX_BT_Data_s bt_tx_data;


void FS_Remote_Control(void);
void Vision_Set_FeedData(void);
static void DummyCmd_Serial_Debug_Send(void);
static void DummyCmd_FKIK_Test_Send(void);
static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd);
void Bt_Set_FeedData(void);

void DummyCmd_Init(void)
{
    DWT_Init(200);
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
    bt_rx_data = BT_Init(&bt_uart_ctrl, &bt_uart_cfg);
#elif (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
    SerialDebug_Init_Config_s serial_debug_config;
    serial_debug_config.usart_config.p_uart_ctrl = &bt_uart_ctrl;
    serial_debug_config.usart_config.p_uart_cfg = &bt_uart_cfg;
    serial_debug_config.pid_callback = NULL;
    serial_debug_instance = SerialDebug_Init(&serial_debug_config);
#endif
    vision_rx_data = Vision_Init(&pc_uart_ctrl, &pc_uart_cfg);
    cmd_kinematic_handle = Dummy_Motormatic_GetKinematicHandle();
    fs_data = FSControlInit(&sbus_ctrl, &sbus_cfg);
    dummy_cmd_pub = PubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));
    dummy_feed_sub = SubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
}

void DummyCmd_Task(void)
{
    SubGetMessage(dummy_feed_sub, &dummy_fetch_data);
    FS_Remote_Control();
    Vision_Set_FeedData();
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
#if DUMMY_CMD_FKIK_TEST_ENABLE
    DummyCmd_FKIK_Test_Send();
#else
    DummyCmd_Serial_Debug_Send();
#endif
#endif

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
    Bt_Set_FeedData();
#endif
    PubPushMessage(dummy_cmd_pub, (void *)&dummy_cmd_send);
}

void FS_Remote_Control(void)
{
    float gripper_angle = map_float_clamp(fs_data[TEMP].knob_l, 0.0f, REMOTE_KNOB_RANGE, 0.0f, 360.0f);

    // 优先级1：R2为急停开关，只要拨到DOWN就直接进入零力模式，其他控制全部不再处理
    if (fs_data[TEMP].switch_r2 == FS_SW_DOWN)
    {
        dummy_cmd_send.arm_mode = ARM_ZERO_FORCE;
        return;
    }
    // R2拨到UP时才允许正常控制，先恢复夹爪的基础控制命令
    else
    {
        // 右旋钮：前半段保持普通夹爪控制，超过一半则启动自动夹爪程序
        if (fs_data[TEMP].knob_r > REMOTE_KNOB_HALF)
        {
            dummy_cmd_send.gripper_mode = GRIPPER_AUTO_GRAB;
        }
        else
        {
            dummy_cmd_send.gripper_mode = GRIPPER_MANUAL_GRAB;
            // 左旋钮：夹爪绝对位置，映射到0~360度，给下游手动夹爪控制使用
            dummy_cmd_send.gripper_current = gripper_angle;
        }
        // 优先级2：R1为强制点位开关，用于快速回到安全预设点
        // R1_DOWN：强行回到0点；R1_MID：强行回到“7”字形点；R1_UP：正常控制
        // 这里使用ARM_PC_MODE下发6个关节目标，避免大小臂模式只控制部分关节
        if (fs_data[TEMP].switch_r1 == FS_SW_DOWN)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = ARM_HOME_JOINT1;
            dummy_cmd_send.joint2_angle = ARM_HOME_JOINT2;
            dummy_cmd_send.joint3_angle = ARM_HOME_JOINT3;
            dummy_cmd_send.joint4_angle = ARM_HOME_JOINT4;
            dummy_cmd_send.joint5_angle = ARM_HOME_JOINT5;
            dummy_cmd_send.joint6_angle = ARM_HOME_JOINT6;
            return;
        }
        else if (fs_data[TEMP].switch_r1 == FS_SW_MID)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = ARM_SEVEN_JOINT1;
            dummy_cmd_send.joint2_angle = ARM_SEVEN_JOINT2;
            dummy_cmd_send.joint3_angle = ARM_SEVEN_JOINT3;
            dummy_cmd_send.joint4_angle = ARM_SEVEN_JOINT4;
            dummy_cmd_send.joint5_angle = ARM_SEVEN_JOINT5;
            dummy_cmd_send.joint6_angle = ARM_SEVEN_JOINT6;
            return;
        }
        // 优先级3：L2为控制方式选择开关
        // L2_DOWN：PC上位机控制，直接使用视觉/上位机给出的目标角度
        if (fs_data[TEMP].switch_l2 == FS_SW_DOWN)
        {
            dummy_cmd_send.arm_mode = ARM_PC_MODE;
            dummy_cmd_send.joint1_angle = -vision_rx_data->joint1;
            dummy_cmd_send.joint2_angle = -vision_rx_data->joint2 + 75.0f;
            dummy_cmd_send.joint3_angle = -vision_rx_data->joint3 + 90.0f;
            dummy_cmd_send.joint4_angle = vision_rx_data->joint4;
            dummy_cmd_send.joint5_angle = -vision_rx_data->joint5;
            dummy_cmd_send.joint6_angle = vision_rx_data->joint6;

        }
        // L2_MID：自定义控制器控制，目前保持自由模式，不叠加遥控器摇杆量
        else if (fs_data[TEMP].switch_l2 == FS_SW_MID)
        {
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_BLUETOOTH)
            dummy_cmd_send.arm_mode = ARM_CUSTOM_MODE;
            dummy_cmd_send.joint1_angle = bt_rx_data->joint1;
            dummy_cmd_send.joint2_angle = bt_rx_data->joint2;
            dummy_cmd_send.joint3_angle = bt_rx_data->joint3;
            dummy_cmd_send.joint4_angle = bt_rx_data->joint4;
            dummy_cmd_send.joint5_angle = bt_rx_data->joint5;
            dummy_cmd_send.joint6_angle = bt_rx_data->joint6;
#else
            dummy_cmd_send.arm_mode = ARM_FREE_MODE;
#endif

        }
        // L2_UP：遥控器控制，L1用于切换大臂/小臂控制
        else if (fs_data[TEMP].switch_l2 == FS_SW_UP)
        {
            dummy_cmd_send.arm_mode = ARM_FREE_MODE;

            // L1_DOWN：小臂控制，四个摇杆以积分方式控制joint3~joint6
            if (fs_data[TEMP].switch_l1 == FS_SW_DOWN)
            {
                dummy_cmd_send.arm_ctrl_mode = SMALL_ARM_CTRL;
                dummy_cmd_send.joint3_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l1);
                dummy_cmd_send.joint4_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l_);
                dummy_cmd_send.joint5_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r1);
                dummy_cmd_send.joint6_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r_);
            }
            // L1_UP：大臂控制，四个摇杆以积分方式控制joint1~joint4
            else
            {
                dummy_cmd_send.arm_ctrl_mode = BIG_ARM_CTRL;
                dummy_cmd_send.joint1_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r_);
                dummy_cmd_send.joint2_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_r1);
                dummy_cmd_send.joint3_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l1);
                dummy_cmd_send.joint4_angle += (float)(REMOTE_JOINT_GAIN * fs_data[TEMP].rocker_l_);
            }
        }
    }
    limit_joint_angles(&dummy_cmd_send);
}

static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd)
{
    // 关节角度限幅 (与 Dummy_Motormatic_Init 中 ArmConfig_t 和 motor_configs 保持一致)
    // joint_min_limit: {-175, -25,   0, -180, -120, -360}
    // joint_max_limit: { 175, 150, 180,  180,  120,  360}

    if (cmd->joint1_angle < -175.0f)
        cmd->joint1_angle = -175.0f;
    if (cmd->joint1_angle > 175.0f)
        cmd->joint1_angle = 175.0f;

    if (cmd->joint2_angle < -25.0f)
        cmd->joint2_angle = -25.0f;
    if (cmd->joint2_angle > 150.0f)
        cmd->joint2_angle = 150.0f;

    if (cmd->joint3_angle < 0.0f)
        cmd->joint3_angle = 0.0f;
    if (cmd->joint3_angle > 180.0f)
        cmd->joint3_angle = 180.0f;

    if (cmd->joint4_angle < -180.0f)
        cmd->joint4_angle = -180.0f;
    if (cmd->joint4_angle > 180.0f)
        cmd->joint4_angle = 180.0f;

    if (cmd->joint5_angle < -120.0f)
        cmd->joint5_angle = -120.0f;
    if (cmd->joint5_angle > 120.0f)
        cmd->joint5_angle = 120.0f;

    if (cmd->joint6_angle < -360.0f)
        cmd->joint6_angle = -360.0f;
    if (cmd->joint6_angle > 360.0f)
        cmd->joint6_angle = 360.0f;
}

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static void DummyCmd_Serial_Debug_Send(void)
{
    static uint16_t send_div_count = 0;

    send_div_count++;
    if (send_div_count < SERIAL_DEBUG_SEND_DIV)
        return;
    send_div_count = 0;

    float channels[SERIAL_DEBUG_JUSTFLOAT_CHANNELS] = {
        dummy_fetch_data.current_joints_feedback.a[0],
        dummy_fetch_data.current_joints_feedback.a[1],
        dummy_fetch_data.current_joints_feedback.a[2],
        dummy_fetch_data.current_joints_feedback.a[3],
        dummy_fetch_data.current_joints_feedback.a[4],
        dummy_fetch_data.current_joints_feedback.a[5],
        dummy_fetch_data.current_pose.X,
        dummy_fetch_data.current_pose.Y,
        dummy_fetch_data.current_pose.Z,
        dummy_fetch_data.current_pose.A,
        dummy_fetch_data.current_pose.B,
        dummy_fetch_data.current_pose.C,
    };

    SerialDebug_Send_JustFloat(serial_debug_instance, channels, SERIAL_DEBUG_JUSTFLOAT_CHANNELS);
}
#if DUMMY_CMD_FKIK_TEST_ENABLE
static void DummyCmd_FKIK_Test_Send(void)
{
    static uint16_t send_div_count = 0;

    send_div_count++;
    if (send_div_count < DUMMY_CMD_FKIK_TEST_SEND_DIV)
        return;
    send_div_count = 0;

    if (cmd_kinematic_handle == NULL)
        return;

    Joint6D_t test_joints = {{30.0f, 20.0f, 40.0f, 10.0f, 20.0f, 30.0f}};
    Pose6D_t test_pose;
    IKSolves_t test_solves;
    Joint6D_t output_joints = test_joints;

    if (!Kinematic_SolveFK(cmd_kinematic_handle, &test_joints, &test_pose))
        return;

    if (Kinematic_SolveIK(cmd_kinematic_handle, &test_pose, &test_joints, &test_solves))
    {
        int best_index = Kinematic_Select_Best_Sol(cmd_kinematic_handle, &test_solves, &test_joints);
        if (best_index >= 0)
            output_joints = test_solves.config[best_index];
    }

    float channels[SERIAL_DEBUG_JUSTFLOAT_CHANNELS] = {
        output_joints.a[0],
        output_joints.a[1],
        output_joints.a[2],
        output_joints.a[3],
        output_joints.a[4],
        output_joints.a[5],
        test_pose.X,
        test_pose.Y,
        test_pose.Z,
        test_pose.A,
        test_pose.B,
        test_pose.C,
    };

    SerialDebug_Send_JustFloat(serial_debug_instance, channels, SERIAL_DEBUG_JUSTFLOAT_CHANNELS);
}
#endif
#endif

void Vision_Set_FeedData(void)
{
    vision_tx_data.header = 0x5A;
    vision_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    vision_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    vision_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    vision_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    vision_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    vision_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    uint8_t all_finished = 1;
    for (int i = 0; i < 6; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0)
        {
            all_finished = 0;
            break;
        }
    }

    // 根据要求，当所有电机完成时设置为11，否则为0
    vision_tx_data.is_finished = all_finished;
    vision_tx_data.tailer = 0XFFFB;
    Vision_Send_Data((uint8_t *)&vision_tx_data, sizeof(Transmit_Data_s));
}

void Bt_Set_FeedData(void)
{
    bt_tx_data.header = 0xAA;
    bt_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    bt_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    bt_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    bt_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    bt_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    bt_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    uint8_t all_finished = 1;
    for (int i = 0; i < 6; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0)
        {
            all_finished = 0;
            break;
        }
    }
    // 根据要求，当所有电机完成时设置为11，否则为0
    bt_tx_data.is_finished = all_finished;
    bt_tx_data.tailer = 0XFFFB;
    BT_SendData((uint8_t *)&bt_tx_data, sizeof(TX_BT_Data_s));
}