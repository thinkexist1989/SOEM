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
        // ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA; //异或操作可以对应1的位置取反，对应0的位置不变，不知道作用是什么
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