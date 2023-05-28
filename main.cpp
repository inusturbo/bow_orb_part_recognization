//
// Created by mashanpeng on 2021/2/11.
//
//standard lib
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <chrono>

#ifdef __cplusplus
extern "C"
{
#endif
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <wiringShift.h>
#include <sys/time.h>
#include <softPwm.h>
#ifdef __cplusplus
}
#endif

// DBoW3
#include "DBoW3/DBoW3.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//namespace
using namespace std;
using namespace cv;
using namespace DBoW3;

int raspi_LCDAddr = 0x27; // LCD 1602 液晶IIC地址 0x27或者0x3F
int raspi_BLEN = 1;       // 写入标志位
int raspi_fd;

#define MAXTIMINGS 85        // 等待最大时间
#define raspi_DHTPIN 0       // DHT11 传感器Pin管脚
#define raspi_Trig 27        // 超声波模块Tring控制管脚
#define raspi_Echo 1         // 超声波模块Echo控制管脚
#define raspi_Led_PinRed 4   // 红色LED 管脚
#define raspi_Led_PinGreen 5 // 绿色LED 管脚
#define raspi_BuzzerPin 26   // 有源蜂鸣器管脚定义

#define uchar unsigned char

int raspi_dht11_dat[5] = {0, 0, 0, 0, 0}; // dht11 数据数组
string get_time_now()
{
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%F %T");
    std::string str = ss.str();
    return str;
}

// LED 初始化
void raspi_led_Init(void)
{
    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 初始化LED灯珠中..." << endl;
    logFile.close();
    softPwmCreate(raspi_Led_PinRed, 0, 100);
    softPwmCreate(raspi_Led_PinGreen, 0, 100);
}

// 设置LED 亮度PWM调节范围是0x00-0xff
void raspi_led_ColorSet(uchar r_val, uchar g_val)
{
    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 打开led灯珠" << endl;
    logFile.close();
    softPwmWrite(raspi_Led_PinRed, r_val);
    softPwmWrite(raspi_Led_PinGreen, g_val);
}

// IIC LCD1602 液晶模块写入字
void raspi_write_word(int data)
{
    int temp = data;
    if (raspi_BLEN == 1)
        temp |= 0x08;
    else
        temp &= 0xF7;
    wiringPiI2CWrite(raspi_fd, temp); //设置IIC LCD1602 液晶模块地址
}
// IIC LCD1602 发送命令
void raspi_send_command(int comm)
{
    int lcd_buf;
    // 首先发送 bit7-4 位
    lcd_buf = comm & 0xF0;
    lcd_buf |= 0x04; // RS = 0, RW = 0, EN = 1
    raspi_write_word(lcd_buf);
    delay(2);
    lcd_buf &= 0xFB; // Make EN = 0
    raspi_write_word(lcd_buf);

    // 其次发送 bit3-0 位
    lcd_buf = (comm & 0x0F) << 4;
    lcd_buf |= 0x04; // RS = 0, RW = 0, EN = 1
    raspi_write_word(lcd_buf);
    delay(2);
    lcd_buf &= 0xFB; // Make EN = 0
    raspi_write_word(lcd_buf);
}

// IIC LCD1602 发送数据
void raspi_send_data(int data)
{
    int lcd_buf;
    // 首先发送 bit7-4 位
    lcd_buf = data & 0xF0;
    lcd_buf |= 0x05; // RS = 1, RW = 0, EN = 1
    raspi_write_word(lcd_buf);
    delay(2);
    lcd_buf &= 0xFB; // Make EN = 0
    raspi_write_word(lcd_buf);

    // 其次发送 bit3-0 位
    lcd_buf = (data & 0x0F) << 4;
    lcd_buf |= 0x05; // RS = 1, RW = 0, EN = 1
    raspi_write_word(lcd_buf);
    delay(2);
    lcd_buf &= 0xFB; // Make EN = 0
    raspi_write_word(lcd_buf);
}

// IIC LCD1602 初始化
void raspi_LCD1602_init()
{
    raspi_send_command(0x33); // 必须先初始化到8线模式码
    delay(5);
    raspi_send_command(0x32); // 然后初始化为4行模式
    delay(5);
    raspi_send_command(0x28); // 2 行 & 5*7 点位
    delay(5);
    raspi_send_command(0x0C); // 启用无光标显示
    delay(5);
    raspi_send_command(0x01); // 清除显示
    wiringPiI2CWrite(raspi_fd, 0x08);
}

// LCD 1602 清空显示函数
void raspi_clear()
{
    raspi_send_command(0x01); // 清除显示
}

// LCD 1602 显示函数
// x为行，y为列
void raspi_write(int lcd_x, int lcd_y, string s)
{
    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 向LCD显示屏写入数据：第" << lcd_x << "行" << lcd_y << "列，写入内容" << s << endl;
    logFile.close();
    const char *data = s.c_str();
    int lcd_addr, lcd_i;
    int lcd_tmp;
    // 选择行与列
    if (lcd_x < 0)
        lcd_x = 0;
    if (lcd_x > 15)
        lcd_x = 15;
    if (lcd_y < 0)
        lcd_y = 0;
    if (lcd_y > 1)
        lcd_y = 1;

    // 移动光标
    lcd_addr = 0x80 + 0x40 * lcd_y + lcd_x;

    raspi_send_command(lcd_addr); //发送地址

    lcd_tmp = strlen(data); // 获取字符串长度
    for (lcd_i = 0; lcd_i < lcd_tmp; lcd_i++)
    {                                 // 依次发送
        raspi_send_data(data[lcd_i]); // 逐一显示
    }
}

// 初始化超声波距离传感器
void raspi_ultraInit(void)
{
    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 初始化超声波传感器" << endl;
    logFile.close();
    pinMode(raspi_Echo, INPUT);  // Echo设置为输入模式
    pinMode(raspi_Trig, OUTPUT); // Tring设置为输出模式
}

// 超声波计算距离函数
double ur_disMeasure(void)
{
    struct timeval ur_tv1;   // 定义时间结构体变量ur_tv1
    struct timeval ur_tv2;   // 定义时间结构体变量ur_tv2
    long ur_time1, ur_time2; // 定义两个长整型时间变量ur_time1,ur_time2
    double ur_dis;           // 定义距离变量

    digitalWrite(raspi_Trig, LOW); // 开始起始
    delayMicroseconds(2);          // 延时2us

    digitalWrite(raspi_Trig, HIGH); // 超声波启动信号，延时10us
    delayMicroseconds(10);          //发出超声波脉冲
    digitalWrite(raspi_Trig, LOW);  // 设置为低电平

    while (!(digitalRead(raspi_Echo) == 1))
        ;                        // 等待回传信号
    gettimeofday(&ur_tv1, NULL); //获取当前时间

    while (!(digitalRead(raspi_Echo) == 0))
        ;                        // 回传信号截止信息
    gettimeofday(&ur_tv2, NULL); //获取当前时间

    ur_time1 = ur_tv1.tv_sec * 1000000 + ur_tv1.tv_usec; // 转换微秒级的时间
    ur_time2 = ur_tv2.tv_sec * 1000000 + ur_tv2.tv_usec; // 转换为微秒级时间
    // 声速在空气中的传播速度为340m/s, 超声波要经历一个发送信号和一个回波信息，
    // 计算公式如下所示：
    ur_dis = (double)(ur_time2 - ur_time1) / 1000000 * 34000 / 2; //求出距离
    string str = to_string(ur_dis) + " cm";
    //raspi_write(0,1,str);
    cout << "Distance is " << str << endl;
    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 获得距离传感器数据：" << str << endl;
    logFile.close();
    return ur_dis; // 返回距离值
}

// 读取dht11温湿度传感器数据函数
void raspi_read_dht11_dat()
{
    uint8_t dht_laststate = HIGH; // 等待高电平
    uint8_t raspi_counter = 0;    // 定义计数值
    uint8_t j = 0, i;
    float f; // 华氏温度

    // 重新给dht11数据数值赋初值
    raspi_dht11_dat[0] = raspi_dht11_dat[1] = raspi_dht11_dat[2] = raspi_dht11_dat[3] = raspi_dht11_dat[4] = 0;

    // DHT11 管脚下拉18ms
    pinMode(raspi_DHTPIN, OUTPUT);   // 设置raspi_DHTPIN 为输出模式
    digitalWrite(raspi_DHTPIN, LOW); // DHT 11 传感器 输出为低电平
    delay(18);                       // 延时18ms
    // 然后上拉40微秒
    digitalWrite(raspi_DHTPIN, HIGH); // DHT 11 传感器 输出为高电平
    delayMicroseconds(40);            // 延时40ms

    // 准备读取DHT11传感器
    pinMode(raspi_DHTPIN, INPUT); // 设置DHT11 传感器为输入模式

    // 检测并读取数据
    // 等待 85 微秒的高电平后进行数据接收
    for (i = 0; i < MAXTIMINGS; i++)
    {
        raspi_counter = 0; // 计数值复位
        while (digitalRead(raspi_DHTPIN) == dht_laststate)
        {                         //等待高电平结束
            raspi_counter++;      // 计数值累加
            delayMicroseconds(1); // 延时1ms
            if (raspi_counter == 255)
            { // 如果等待的时间过长，则退出
                break;
            }
        }
        dht_laststate = digitalRead(raspi_DHTPIN); // 重新读取DTH11电平信号
        if (raspi_counter == 255)
            break; // 如果等待的时间过长，则退出

        // 忽略前三个数据
        if ((i >= 4) && (i % 2 == 0))
        {
            // 将每位放入存储字节中
            raspi_dht11_dat[j / 8] <<= 1;
            if (raspi_counter > 16)
                raspi_dht11_dat[j / 8] |= 1;
            j++;
        }
    }
    // 检查我们读取40位(8bit x 5) +校验校验和在最后一个字节打印出来
    // 如果数据是正常的！
    if ((j >= 40) &&
        (raspi_dht11_dat[4] == ((raspi_dht11_dat[0] + raspi_dht11_dat[1] + raspi_dht11_dat[2] + raspi_dht11_dat[3]) & 0xFF)))
    {
        f = raspi_dht11_dat[2] * 9. / 5. + 32; // 计算温度值
        // 打印出温度值和湿度值
        printf("Humidity = %d.%d %% Temperature = %d.%d *C (%.1f *F)\n",
               raspi_dht11_dat[0], raspi_dht11_dat[1], raspi_dht11_dat[2], raspi_dht11_dat[3], f);
        string s = "HUM" + to_string(raspi_dht11_dat[0]) + "." + to_string(raspi_dht11_dat[1]) + " TEM" + to_string(raspi_dht11_dat[2]) + "." + to_string(raspi_dht11_dat[3]);
        raspi_write(0, 0, s);
        ofstream logFile;
        logFile.open("log.txt", ios::app);
        logFile << "[" << get_time_now() << "] "
                << "### 获得温度湿度数据：" << s << endl;
        logFile.close();
    }
}

/*
 * 功能：读取某个文本文件中所有的文件的名称
 * 输入：文件列表文件路径
 * 返回：文件路径的容器
 * 作者：马善鹏
 * 日期：2021.2.11
*/
std::vector<std::string> readImageName(std::string imageNameFile)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 读取图片名称到vector容器中" << endl;
    logFile.close();
    std::vector<std::string> v_imageName;
    std::fstream file(imageNameFile);
    std::string temp;
    while (getline(file, temp))
    {
        v_imageName.push_back(temp);
    }
    return v_imageName;
}

/*
 * 功能：获得某个图像文件的描述子
 * 输入：图片文件路径
 * 返回：描述子矩阵作者：马善鹏
 * 日期：2021.2.11
*/
cv::Mat getDescriptor(std::string ImageName, std::string descriptorName)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 获取图片" << ImageName << "的" << descriptorName << "特征。" << endl;
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptorName == "ORB")
        fdetector = cv::ORB::create();
    else if (descriptorName == "BRISK")
        fdetector = cv::BRISK::create();
    else if (descriptorName == "AKAZE")
        fdetector = cv::AKAZE::create();
    else
        throw std::runtime_error("Invalid descriptor");
    assert(!descriptorName.empty());

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    logFile << "[" << get_time_now() << "] "
            << "### 读取图片：" << ImageName << std::endl;
    std::cout << ">>> reading image: " << ImageName << std::endl;
    //读取图像
    cv::Mat image = cv::imread(ImageName, 0);
    if (image.empty())
        throw std::runtime_error("!!! Could not open image" + ImageName);
    std::cout << ">>> extracting features" << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在提取特征" << std::endl;
    clock_t time_stt = clock();

    fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    cout << "This image has " << keypoints.size() << " points." << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 这幅图片提取出" << keypoints.size() << "个特征点" << std::endl;
    cout << "### done detecting features" << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "提取结束，用时" <<
        1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
    cout << "### extracting feature time is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile.close();

    return descriptors;
}

cv::Mat getDescriptorFromMat(cv::Mat image, std::string descriptorName)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 获取图片的" << descriptorName << "特征。" << endl;
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptorName == "ORB")
        fdetector = cv::ORB::create();
    else if (descriptorName == "BRISK")
        fdetector = cv::BRISK::create();
    else if (descriptorName == "AKAZE")
        fdetector = cv::AKAZE::create();
    else
        throw std::runtime_error("Invalid descriptor");
    assert(!descriptorName.empty());

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::cout << ">>> reading image: " << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在读取图片..." << endl;
    //读取图像
    if (image.empty())
        throw std::runtime_error("!!! Could not open image");
    std::cout << ">>> extracting features" << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在提取特征..." << endl;
    clock_t time_stt = clock();

    fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    cout << "This image has " << keypoints.size() << " points." << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 图像中含有" << keypoints.size() << "个特征点" << endl;
    std::cout << "### done detecting features" << std::endl;

    cout << "### extracting feature time is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 提取结束，用时" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile.close();
    return descriptors;
}

map<int, string> readPartIdImageIdMap(string mapfilepath)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 读取零件ID与图片ID列表:" << mapfilepath << endl;
    ifstream ins(mapfilepath);
    map<int, string> your_map;
    while (!ins.eof())
    {
        int key;
        string value;
        ins >> key >> value;
        your_map.insert(make_pair(key, value));
    }
    //for (auto itr = your_map.begin(); itr != your_map.end(); itr++) {
    //    cout << "The partID of " << itr->first << "th image is " << itr->second << endl;
    //}
    logFile.close();
    return your_map;
}

map<string, int> readPartNameMap(string mapfilepath)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 读取零件名称列表:" << mapfilepath << endl;
    ifstream ins(mapfilepath);
    map<string, int> your_map;
    while (!ins.eof())
    {
        int key = 0;
        string value;
        ins >> value;
        your_map.insert(make_pair(value, key));
    }
    //for (auto itr = your_map.begin(); itr != your_map.end(); itr++) {
    //    cout << "The partID of " << itr->first << "th image is " << itr->second << endl;
    //}
    logFile.close();
    return your_map;
}

string findImageIdFromMap(map<int, string> imagemap, int imageId)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 查找图片ID" << imageId << "对应的零件名称" << endl;
    map<int, string>::iterator iterr;
    iterr = imagemap.find(imageId);
    if (iterr != imagemap.end())
    {
        string str = iterr->second;
        cout << "Find, the partID is " << str << endl;
        logFile << "[" << get_time_now() << "] "
                << "### 查找成功，零件名称为" << str << endl;
        return iterr->second;
    }
    else
    {
        logFile << "[" << get_time_now() << "] "
                << "### 查找失败" << endl;
    }
    logFile.close();
}

void saveImageIdPartIdMap(map<int, string> imagePartIdMap, string savefilepath)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 保存图片ID与零件ID的列表" << endl;
    ofstream ous(savefilepath);
    auto iter = imagePartIdMap.begin();
    for (; iter != imagePartIdMap.end(); iter++)
    {
        ous << iter->first << " " << iter->second << endl;
    }
    logFile.close();
}

/*
功能：获得文件列表中所有图像文件的描述子
输入：图片文件路径
返回：描述子矩阵
作者：马善鹏
日期：2021.2.11
*/
vector<cv::Mat> getDescriptors(std::vector<string> path_to_images, std::string descriptorName)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 获得图片的" << descriptorName << "特征" << endl;
    clock_t time_total = clock();
    std::vector<cv::Mat> features;
    for (auto &ImageName : path_to_images)
    {
        features.push_back(getDescriptor(ImageName, descriptorName));
    }
    cout << "### total time is " << 1000 * (clock() - time_total) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 用时" << 1000 * (clock() - time_total) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile.close();
    return features;
}

void vocCreationAndTraining(std::vector<cv::Mat> &features)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 创建和训练词典中" << endl;
    const int k = 7;
    const int L = 5;
    const DBoW3::WeightingType weight = DBoW3::TF_IDF;
    const DBoW3::ScoringType score = DBoW3::L1_NORM;
    DBoW3::Vocabulary voc(k, L, weight, score);
    logFile << "[" << get_time_now() << "] "
            << "### 创建" << k << "^" << L << "的词典中" << endl;
    std::cout << ">>> Creating a " << k << "^" << L << " vocabulary..." << std::endl;
    voc.create(features);
    logFile << "[" << get_time_now() << "] "
            << "### 创建结束" << endl;
    std::cout << "### done!" << std::endl;
    cout << "### Vocabulary information: " << endl
         << voc << endl
         << endl;
    // save the vocabulary to disk
    cout << endl
         << ">>> Saving vocabulary..." << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 保存词典中" << endl;
    voc.save("voc.yml.gz");
    cout << "### voc Creation And Training Done" << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 保存和训练词典结束" << endl;
    logFile.close();
}

void databaseCreation(vector<cv::Mat> &features)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 创建词袋数据库中" << endl;
    std::cout << ">>> Creating a database..." << std::endl;

    // load the vocabulary from disk
    DBoW3::Vocabulary voc("voc.yml.gz");

    DBoW3::Database db(voc, true, 2);
    for (size_t i = 0; i < features.size(); i++)
        db.add(features[i]);
    logFile << "[" << get_time_now() << "] "
            << "### 数据库的信息" << endl
            << db << std::endl;
    std::cout << "### Database information: " << std::endl
              << db << std::endl;
    std::cout << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 保存数据库中" << endl;
    std::cout << ">>> Saving database..." << std::endl;
    db.save("db.yml.gz");
    logFile << "[" << get_time_now() << "] "
            << "### 保存成功" << endl;
    std::cout << "### database Creation done!" << std::endl;
    logFile.close();
}

void databaseDetec(cv::Mat &feature)
{

    ofstream logFile;
    logFile.open("log.txt", ios::app);
    logFile << "[" << get_time_now() << "] "
            << "### 正在加载数据库" << endl;
    std::cout << ">> Loading database ..." << std::endl;
    clock_t time_stt = clock();
    DBoW3::Database db("db.yml.gz");
    logFile << "[" << get_time_now() << "] "
            << "### 加载数据库成功，数据库信息如下：" << endl
            << db << std::endl;
    std::cout << "### Loading database done! This is: " << std::endl
              << db << std::endl;
    cout << "### Loading time is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 加载数据库用时：" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "### Querying the database: " << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在查询数据库" << endl;
    QueryResults ret;
    db.query(feature, ret, 4);

    //ret[0] is always the same image in this case, because we added it to the
    // database. ret[1] is the second best match.

    if (ret.empty())
    {
        logFile << "[" << get_time_now() << "] "
                << "### 没有查找到相似的图片" << endl;
        std::cout << "!!! Not have similar image!";
    }

    std::cout << ret << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "=== 最相似的图片 ID 是 " << ret[0].Id << " 相似分数是 " << ret[0].Score << endl;
    logFile << "[" << get_time_now() << "] "
            << "=== 第二相似的图片 ID 是 " << ret[1].Id << " 相似分数是 " << ret[1].Score << endl;
    std::cout << "=== The best matching image ID is " << ret[0].Id << " and the score is " << ret[0].Score << std::endl;
    std::cout << "=== The second matching image ID is " << ret[1].Id << " and the score is " << ret[1].Score
              << std::endl;

    std::cout << std::endl;
    cout << endl;
    logFile.close();
}

DBoW3::Database loadDatabase()
{
    ofstream logFile;
    logFile << "[" << get_time_now() << "] "
            << "### 正在加载数据库，请稍候..." << endl;
    std::cout << ">>> Loading database. It will take many time, please wait." << std::endl;
    clock_t time_stt = clock();
    DBoW3::Database db("db.yml.gz");
    logFile << "[" << get_time_now() << "] "
            << "### 加载时间数据库是" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "### Loading time is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile.close();
    return db;
}

int databaseContiDetec(cv::Mat &feature, DBoW3::Database &db)
{
    ofstream logFile;
    std::cout << ">> Loading database ..." << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在加载数据库" << endl;
    std::cout << "### Loading database done! This is: " << std::endl
              << db << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 加载数据库成功，信息如下：" << endl
            << db << std::endl;
    std::cout << "Querying the database: " << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 正在从数据库查询：" << endl;
    clock_t time_stt = clock();
    QueryResults ret;
    db.query(feature, ret, 4);

    //ret[0] is always the same image in this case, because we added it to the
    // database. ret[1] is the second best match.

    if (ret.empty())
    {
        logFile << "[" << get_time_now() << "] "
                << "!!! 没有相似的图片！" << endl;
        std::cout << std::endl
                  << std::endl
                  << "!!! Not have similar image!" << std::endl
                  << std::endl;
        return -1;
    }

    if (ret[0].Score < 0.1)
    {
        logFile << "[" << get_time_now() << "] "
                << "!!! 没有相似的图片！" << endl;
        std::cout << std::endl
                  << std::endl
                  << "!!! Not have similar image!" << std::endl
                  << std::endl;
        return -1;
    }
    string str1 = findImageIdFromMap(readPartIdImageIdMap("text.txt"), ret[0].Id);
    string str2 = findImageIdFromMap(readPartIdImageIdMap("text.txt"), ret[1].Id);
    if (str1 != str2)
    {
        logFile << "[" << get_time_now() << "] "
                << "!!! 没有相似的图片！" << endl;
        std::cout << std::endl
                  << std::endl
                  << "!!! Not have similar image!" << std::endl
                  << std::endl;
        return -1;
    }

    std::cout << ret << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "=== 最相似的图片 ID 是 " << ret[0].Id << " 相似分数是 " << ret[0].Score << endl;
    logFile << "[" << get_time_now() << "] "
            << "=== 第二相似的图片 ID 是 " << ret[1].Id << " 相似分数是 " << ret[1].Score << endl;
    std::cout << "=== The best matching image ID is " << ret[0].Id << " and the score is " << ret[0].Score
              << std::endl;
    std::cout << "=== The second matching image ID is " << ret[1].Id << " and the score is " << ret[1].Score
              << std::endl;

    std::cout << std::endl;
    logFile << "[" << get_time_now() << "] "
            << "### 查询用时：" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "### Query time is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    logFile.close();
    return ret[0].Id;
}

void trainDatabase(string descriptorName, string imageNameFile)
{
    std::vector<cv::Mat> features;
    std::vector<std::string> filenames;
    filenames = readImageName(imageNameFile);
    features = getDescriptors(filenames, descriptorName);
    vocCreationAndTraining(features);
    databaseCreation(features);
}

//1=append,0=new
void trainFromCamera(string mapfilepath, int flag = 0)
{
    ofstream logFile;
    ofstream OpenFile;
    ofstream partFile;
    if (flag == 1)
    {
        logFile << "[" << get_time_now() << "] "
                << "### 以追加模式训练" << endl;
        OpenFile.open("list.txt", ios::app);
        partFile.open("partName.txt", ios::app);
    }
    else
    {
        logFile << "[" << get_time_now() << "] "
                << "### 以全新模式训练" << endl;
        OpenFile.open("list.txt");
        partFile.open("partName.txt");
    }
    if (OpenFile.fail())
    {
        logFile << "[" << get_time_now() << "] "
                << "### 打开文件错误" << endl;
        cout << "打开文件错误!" << endl;
        exit(0);
    }
    map<int, string> yourmap;
    if (flag == 1)
        map<int, string> yourmap = readPartIdImageIdMap(mapfilepath);
    string writePath = "../TrainingImage/";
    logFile << "[" << get_time_now() << "] "
            << "### 打开摄像头" << endl;
    VideoCapture capture(0);
    string name;
    namedWindow("hello", WINDOW_AUTOSIZE);
    int i = 0;
    if (flag == 1)
    {
        logFile << "[" << get_time_now() << "] "
                << "### 加载上次拍摄时的图像ID，以便与追加训练" << endl;
        ifstream idfile("beginId.txt");
        idfile >> i;
    }
    while (true)
    {
        cout << "请输入零件编号，按回车确定:" << endl;
        string partId;
        cin >> partId;
        cout << "您输入的编号是：" << partId << endl;
        partFile << partId << endl;
        logFile << "[" << get_time_now() << "] "
                << "### 用户输入零件名称" << partId << endl;
        cout << "请您按下【空格键】执行拍摄，按【a】键拍摄下个零件，按【q】结束拍摄，开始训练词袋。" << endl;
        while (true)
        {
            Mat framesrc;
            Mat frame;
            capture >> framesrc;
            flip(framesrc, frame, -1);
            if (32 == waitKey(20))
            {
                name = writePath + to_string(i) + ".jpg";
                imwrite(name, frame);
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                cout << name << endl;
                OpenFile << name << endl;
                yourmap.insert(make_pair(i, partId));
                i++;
                ofstream beginIdfile("beginId.txt");
                if (beginIdfile.fail())
                {
                    cout << "打开文件错误!" << endl;
                    exit(0);
                }
                beginIdfile << i;
                beginIdfile.close();
                logFile << "[" << get_time_now() << "] "
                        << "### 拍摄照片：" << name << endl;
            }
            if (97 == waitKey(10))
            {
                logFile << "[" << get_time_now() << "] "
                        << "### 拍摄当前零件结束" << endl;
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                delay(100);
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                break;
            }
            if (113 == waitKey(10))
            {
                logFile << "[" << get_time_now() << "] "
                        << "### 拍摄过程结束" << endl;
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                delay(100);
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                delay(100);
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(100);                          // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                goto back;
            }

            imshow("hello", frame);
        }
    }
back:
    saveImageIdPartIdMap(yourmap, mapfilepath);
    OpenFile.close();
    partFile.close();
    trainDatabase("ORB", "list.txt");
    logFile.close();
}

int cameraDetec(string descriptorName)
{
    ofstream logFile;
    logFile << "[" << get_time_now() << "] "
            << "### 开始检测过程！！！" << endl;
    logFile << "[" << get_time_now() << "] "
            << "### 开启摄像头" << endl;
    cv::VideoCapture capture(0);
    DBoW3::Database db = loadDatabase();
    namedWindow("hello", WINDOW_AUTOSIZE);
    map<string, int> partmap = readPartNameMap("partName.txt");
    while (true)
    {
        Mat framesrc;
        Mat frame;
        if (!capture.read(framesrc))
        {
            logFile << "[" << get_time_now() << "] "
            << "### 读取视频失败" << endl;
            cout << "！！！ 读取视频失败" << endl;
            return -1;
        }
        flip(framesrc, frame, -1);
        double ur_dis;            //定义一个局部变量，保存距离值
        raspi_read_dht11_dat();   // 读取dht11温湿度传感器数据函数
        ur_dis = ur_disMeasure(); //获取超声波计算距离
        if (ur_dis < 29.99)
        {
            logFile << "[" << get_time_now() << "] "
            << "### 距离传感器检测到距离太近，无法识别" << endl;
            raspi_led_ColorSet(0xff, 0x00); //红色
            raspi_write(0, 1, "TOO    CLOSE !!!!");
        }
        else
        {
            raspi_led_ColorSet(0xff, 0x00); //绿色
            Mat descriptor = getDescriptorFromMat(frame, descriptorName);

            int i = databaseContiDetec(descriptor, db);
            if (i == -1)
            {
                logFile << "[" << get_time_now() << "] "
                    << "### 没有匹配的图像" << endl;
                //raspi_clear();
                raspi_write(0, 1, "                ");
                raspi_write(0, 1, "Cannot Find Any");
                //delay(1000);
                cout << "！！！ 没有匹配的图像！" << endl;
                continue;
            }
            else
            {
                string st = findImageIdFromMap(readPartIdImageIdMap("text.txt"), i);
                cout << st << endl;
                logFile << "[" << get_time_now() << "] "
                    << "### 检测到物体" << st << endl;
                std::vector<KeyPoint> keypoint;
                Ptr<FeatureDetector> detector = ORB::create();
                detector->detect(frame, keypoint);
                Mat outimg;
                drawKeypoints(frame, keypoint, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                putText(outimg, st, Point(10, 50), font_face, 2, cv::Scalar(0, 0, 255), 5, 8, 0);

                imshow("hello", outimg);
                waitKey(10);
                raspi_write(0, 1, "                ");
                raspi_write(0, 1, st);
                raspi_led_ColorSet(0x00, 0xff);      //绿色
                digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
                delay(70);                           // 延时100ms
                digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸟器
                partmap[st] = 1;
                ofstream isDetecFile;
                isDetecFile.open("isDetec.txt");
                for (auto itr = partmap.begin(); itr != partmap.end(); itr++)
                {
                    isDetecFile << itr->first << " " << itr->second << endl;
                }
                isDetecFile.close();
            }
        }
    }
    logFile.close();
}

int main()
{
    //training a database
    //trainDatabase("ORB","name.txt");

    //detect a image
    //DBoW3::Database db = loadDatabase();
    //Mat read1 = getDescriptor("49.jpg","ORB");
    //databaseContiDetec(read1,db);
    //Mat read2 = getDescriptor("80.jpg","ORB");
    //databaseContiDetec(read2,db);
    if (wiringPiSetup() == -1)
        exit(1);
    raspi_fd = wiringPiI2CSetup(raspi_LCDAddr); //初始化地址
    raspi_LCD1602_init();                       //初始化显示屏
    raspi_ultraInit();                          // 调用超声波模块初始化工作
    raspi_led_Init();
    pinMode(raspi_BuzzerPin, OUTPUT);    // 有源蜂鸣器设置为输出模式
    digitalWrite(raspi_BuzzerPin, LOW);  // 蜂鸣器为低电平触发，所以使能蜂鸣器让其发声
    delay(1000);                         // 蜂鸣1秒
    digitalWrite(raspi_BuzzerPin, HIGH); // 蜂鸣器设置为高电平，关闭蜂鸣器
    cout << "TRAIN press 1, Detect press 2" << endl;
    int id;
    cin >> id;
    if (id == 1)
    {
        int mode;
        cout << "APPEND press 1, a new Scanning press 0" << endl;
        cin >> mode;
        trainFromCamera("text.txt", mode);
    }
    else
    {
        cameraDetec("ORB");
    }

    //trainFromCamera("text.txt");

    //map<int ,string> imagePartIdMap;
    //string word;
    //int count=0;
    //imagePartIdMap.insert(make_pair(0,"A0001"));
    //imagePartIdMap.insert(make_pair(3,"A0002"));

    //ofstream ous("text.txt");
    //auto iter=imagePartIdMap.begin();
    //for(;iter!=imagePartIdMap.end();iter++){
    //    ous<<iter->first<<" "<<iter->second<<endl;
    //}
    //map<int ,string> yourmap = readPartIdImageIdMap("text.txt");

    //string ins = findImageIdFromMap(yourmap,0);

    return 0;
}
