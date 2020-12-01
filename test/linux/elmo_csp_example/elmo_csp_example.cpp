#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/time.h> //系统时间函数
#include <pthread.h>  // 线程函数
#include <unistd.h>
#include <math.h>

#include <ethercat.h> // soem 头文件 里面包含了相关头文件

/* 函数声明 */
void *ecatcheck(void *ptr);        //etherCAT错误检查
void CspTest(std::string &ifname); //csp测试

/* 宏定义 */
#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

struct PositionOut
{
    int32 targetPosition;
    int32 digitalOutputs;
    int16 controlWord;
};

struct PositionIn
{
    int32 positionActualValue;
    int32 digitalInputs;
    int16 statusWord;
};

/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

#define CHECKERROR(slaveId)                                                                                                                                                                       \
    {                                                                                                                                                                                             \
        ec_readstate();                                                                                                                                                                           \
        printf("EC> \"%s\" %x - %x [%s] \n", (char *)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
    }

/* 全局变量 */
char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

using namespace std;

/*
 * 主函数
 */
int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\n Elmo CST Example \n");

    string ifname;

    if (argc > 1)
    {
        ifname = string(argv[1]);
    }
    else
    {
        ifname = "enp6s0";
        printf("Usage: elmo_cst_example ifname\n ifname is default to enp6s0.\n");
    }

    /* create thread to handle slave error handling in OP */
    pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime); // (void) &ctime
    /* start cyclic part */
    CspTest(ifname);

    return (0);
}

/* 
 * Elmo csp模式测试
 */
void CspTest(std::string &ifname)
{
    if (ec_init(ifname.c_str()) <= 0)
    {
        cout << "No socket connection on " << ifname << endl;
        cout << "Execute as root." << endl;
        return;
    }

    cout << "ec_init on " << ifname << " succeeded." << endl; //初始化成功

    if (ec_config_init(FALSE) == 0) // 配置初始化并返回检测到的从站数量(可以通过返回的datagram确认)
    {
        cout << "No slave found！" << endl;
        return;
    }

    for (int i = 1; i <= ec_slavecount; i++) //循环 ec_slave[0]保留为master，通过调用ec_config()填充
    {
        // printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
        printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n Has CA: %s\n",
               i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
               ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");

        /** CompleteAccess disabled for Elmo driver */
        // ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA; //TODO: 异或操作可以对应1的位置取反，对应0的位置不变，不知道作用是什么？
    }

    /* 检查slave状态是否全部为Pre-Op */
    if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP)
    {
        cout << "EtherCAT state is not Pre-Op" << endl;
        return;
    }
    else
        cout << "EtherCAT state is Pre-Op" << endl;

    if (ec_configdc() == FALSE) // Locate DC slaves, measure propagation delays.
        cout << "Config DC Failed!" << endl;
    else
        cout << "Config DC Succeeded!" << endl;

    /** set PDO mapping */
    /** opMode: 8  => Position profile */

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

    for (int i = 1; i <= ec_slavecount; i++)
    {
        WRITE(i, 0x6060, 0, buf8, 8, "OpMode"); //配置模式 8->csp
        READ(i, 0x6061, 0, buf8, "OpMode display");

        READ(i, 0x1c12, 0, buf32, "rxPDO:0");
        READ(i, 0x1c13, 0, buf32, "txPDO:0");

        READ(i, 0x1c12, 1, buf32, "rxPDO:1");
        READ(i, 0x1c13, 1, buf32, "txPDO:1");
    }

    int32 ob2;
    int os;

    for (int i = 1; i <= ec_slavecount; i++)
    {
        /* Map velocity PDO assignment via Complete Access*/
        uint16 map_1c12[4] = {0x0001, 0x1600};
        uint16 map_1c13[3] = {0x0001, 0x1a00};

        ec_SDOwrite(i, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        ec_SDOwrite(i, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);

        // os = sizeof(ob2);
        // ob2 = 0x16020001;
        // ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
        // os = sizeof(ob2);
        // ob2 = 0x1a020001;
        // ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

        READ(i, 0x1c12, 0, buf32, "rxPDO:0");
        READ(i, 0x1c13, 0, buf32, "txPDO:0");

        READ(i, 0x1c12, 1, buf32, "rxPDO:1");
        READ(i, 0x1c13, 1, buf32, "txPDO:1");
    }

    /** if CA disable => automapping works (TODO: 如果没有这句话，驱动器状态不会变成Safe-Op，不知道为什么) */
    ec_config_map(&IOmap);

    /** disable heartbeat alarm */
    for (int i = 1; i <= ec_slavecount; i++)
    {
        // READ(i, 0x10F1, 2, buf32, "Heartbeat?"); //0x10F1 DS301-p215
        // WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

        WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
        WRITE(i, 0x2f75, 0, buf16, 10, "Interpolation timeout");
    }

    /* 检查slave状态是否全部为Safe-Op */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
    {
        cout << "EtherCAT state is not Safe-Op" << endl;
        return;
    }
    else
        cout << "EtherCAT state is Safe-Op" << endl;

    printf("Request operational state for all slaves\n");
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /** going operational */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* To enter state OP we need to send valid data to outputs. The EtherCAT frame handling is split into ec_send_processdata and ec_receive_processdata. */
    /* send one valid process data to make outputs in slaves happy*/
    int wkc = ec_send_processdata();
    cout << "ec_send_processdata returned wkc is: " << wkc << endl;
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    cout << "ec_receive_processdata returned wkc is: " << wkc << endl; //此处wkc为1，是因为处于Safe-Op，从站只能Tx

    for (int i = 1; i <= ec_slavecount; i++)
    {
        // WRITE(i, 0x6081, 0, buf32, 10000, "Profile velocity");
        // WRITE(i, 0x607F, 0, buf32, 100000, "Max profile velocity");

        // WRITE(i, 0x6083, 0, buf32, 4000, "Profile acceleration");
        // WRITE(i, 0x6084, 0, buf32, 4000, "Profile deceleration");

        // WRITE(i, 0x60C5, 0, buf32, 10000, "Max acceleration");
        // WRITE(i, 0x60C6, 0, buf32, 10000, "Max deceleration");

        // READ(i, 0x6081, 0, buf32, "Profile velocity");
        // READ(i, 0x607F, 0, buf32, "Max profile velocity");

        // READ(i, 0x6083, 0, buf32, "Profile acceleration");
        // READ(i, 0x6084, 0, buf32, "Profile deceleration");

        // READ(i, 0x60C5, 0, buf32, "Max acceleration");
        // READ(i, 0x60C6, 0, buf32, "Max deceleration");

        // READ(i, 0x60C2, 0, buf32, "Max deceleration");

        READ(i, 0x1c32, 2, buf32, "SM2 Cycle Time");
        READ(i, 0x1c33, 2, buf32, "SM3 Cycle Time");
    }

    /* request OP state for all slaves */
    ec_writestate(0); //将ec_slave[i].state状态写入各个slave
    int chk = 40;
    /* wait for all slaves to reach OP state */
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        cout << "Operational state reached for all slaves." << endl;
    }
    else
    {
        cout << "Not all slaves reached operational state." << endl;
        return;
    }

    /**
     * Drive state machine transistions
     *   0 -> 6 -> 7 -> 15
     */
    for (int i = 1; i <= ec_slavecount; i++)
    {
        READ(i, 0x6041, 0, buf16, "*status word*");
        if (buf16 == 0x218)
        {
            WRITE(i, 0x6040, 0, buf16, 128, "*control word*");
            usleep(100000);
            READ(i, 0x6041, 0, buf16, "*status word*");
        }

        WRITE(i, 0x6040, 0, buf16, 0, "*control word*");
        usleep(100000);
        READ(i, 0x6041, 0, buf16, "*status word*");

        WRITE(i, 0x6040, 0, buf16, 6, "*control word*");
        usleep(100000);
        READ(i, 0x6041, 0, buf16, "*status word*");

        WRITE(i, 0x6040, 0, buf16, 7, "*control word*");
        usleep(100000);
        READ(i, 0x6041, 0, buf16, "*status word*");

        WRITE(i, 0x6040, 0, buf16, 15, "*control word*");
        usleep(100000);
        READ(i, 0x6041, 0, buf16, "*status word*");

        CHECKERROR(i);
        // READ(i, 0x1a0b, 0, buf32, "OpMode Display PDO Mappings");
        READ(i, 0x6061, 0, buf8, "OpMode Display");

        READ(i, 0x1001, 0, buf8, "Error");
    }

    cout << "ec_state is: " << ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) << endl;

    /* 可以开始工作了 */
    PositionOut *target = (struct PositionOut *)(ec_slave[1].outputs);
    PositionIn *val = (struct PositionIn *)(ec_slave[1].inputs);

    int posss = 0;
    int reachedIntial = 0;

    while (1)
    {
        /** PDO I/O refresh */
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        // cout << "wkc is: " << wkc << endl;
        if (wkc < expectedWKC)
            return;

        switch (target->controlWord)
        {
        case 0:
            cout << "control word is 0" << endl;
            target->controlWord = 6;
            break;
        case 6:
            cout << "control word is 6" << endl;
            target->controlWord = 7;
            break;
        case 7:
            cout << "control word is 7" << endl;
            target->controlWord = 15;
            // usleep(100000);
            break;
        case 128:
            cout << "control word is 128" << endl;
            target->controlWord = 0;
            break;
        default:
            if (val->statusWord >> 3 & 0x01)
            {
                READ(1, 0x1001, 0, buf8, "Error");
                target->controlWord = 128;
            }
        }

        if ((val->statusWord & 0x0fff) == 0x0237)
        {
            // target->controlWord |= 0x10;
            target->targetPosition = val->positionActualValue + 100;
            posss += 200;
        }

        printf("Position target is: %d, Position actual is: %d, Control word is: 0x%x, Status word is: 0x%x               ", target->targetPosition, val->positionActualValue, target->controlWord, val->statusWord);
        printf("\r");

        usleep(1000);
    }

    /* strop SOEM, close socket */
    ec_close();
}

/* 
 * EtherCAT 错误检查
 * 需要单开一路线程循环检测是否出现错误
 */
void *ecatcheck(void *ptr)
{
    (void)ptr;
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf(".");
        }
        usleep(250);
    }
}