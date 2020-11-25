#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/time.h> //系统时间函数
#include <pthread.h>  // 线程函数
#include <unistd.h>

#include <ethercat.h> // soem 头文件 里面包含了相关头文件

/* 函数声明 */
void *ecatcheck(void *ptr);        //etherCAT错误检查
void CspTest(std::string &ifname); //csp测试

/* 宏定义 */
#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

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
        WRITE(i, 0x6060, 0, buf8, 8, "OpMode");
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
        uint16 map_1c12[4] = {0x0003, 0x1601, 0x1602, 0x1604};
        uint16 map_1c13[3] = {0x0002, 0x1a01, 0x1a03};

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

    /* 检查slave状态是否全部为Safe-Op */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
    {
        cout << "EtherCAT state is not Safe-Op" << endl;
        return;
    }
    else
        cout << "EtherCAT state is Safe-Op" << endl;

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