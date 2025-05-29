#include <stdint.h>
#include <iostream>
#include "ns3/buffer.h"
#include "ns3/address-utils.h"
#include "ns3/log.h"
#include "mp-quic-typedefs.h"
#include <stdlib.h>
#include <queue>
#include "ns3/traced-value.h"
#include "ns3/trace-source-accessor.h"
#include <ns3/object-base.h>
#include "ns3/simulator.h"
#include "time.h"
#include "ns3/string.h"

#include "quic-socket-tx-scheduler.h"
#include "quic-socket-base.h"

#include <algorithm>
#include <math.h>
#include <cmath>
#include <boost/assign/list_of.hpp>

NS_LOG_COMPONENT_DEFINE ("MpQuicTypeDefs");
namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MpQuicSubFlow);

/**
 * zhiy zeng: 在NS3中注册一个名为MpQuicSubFlow的类
 * 定义MPQUIC中子流的一些基本属性和追踪源，以便
 * 在运行时进行配置和监控
 */
TypeId
MpQuicSubFlow::GetTypeId (void)
{
    // 定义一个静态的TypeId对象, 对应的是类MpQuicSubFlow, 位于命名空间ns3中
    static TypeId tid = TypeId ("ns3::MpQuicSubFlow")
        .SetParent (Object::GetTypeId ()) // 继承自Object类, 这是ns3中所有可管理对象的基类
        .AddAttribute ("CCType", // 添加名为CCType的属性
                   "congestion control type", // 属性描述：拥塞控制算法类型
                   StringValue ("new"), // 默认值为字符串"new"
                   // 关联到成员变量m_ccType, 可通过m_ccType访问
                   MakeStringAccessor (&MpQuicSubFlow::m_ccType),
                   // MakeStringChecker()用于检查字符串的合法性
                   MakeStringChecker ())
        .AddAttribute ("delay",
                   "congestion control type", // 属性描述：延迟?
                   DoubleValue (0.04), // 默认值为0.04
                   // 关联到成员变量m_delay, 可通过m_delay访问
                   MakeDoubleAccessor (&MpQuicSubFlow::m_delay),
                     // MakeDoubleChecker()用于检查double类型的合法性：是否大于等于0
                   MakeDoubleChecker<double> (0))
        .AddTraceSource ("SubflowCwnd", // 添加名为SubflowCwnd的追踪源, 追踪子流拥塞窗口大小
                       "An integer value to trace.", // 追踪整数值
                       // 关联到成员变量m_cWnd, 可通过m_cWnd访问
                       MakeTraceSourceAccessor (&MpQuicSubFlow::m_cWnd),
                       "ns3::TracedValueCallback::Int32") // 追踪整数值, 回调类型是ns3::...::Int32
        .AddTraceSource ("Throughput", // 添加名为Throughput的追踪源, 追踪吞吐量
                       "An integer to trace.",
                       MakeTraceSourceAccessor (&MpQuicSubFlow::m_throughputBps),
                       "ns3::TracedValueCallback::double")
        .AddTraceSource ("RTT", // 添加名为RTT的追踪源, 追踪RTT
                       "An integer to trace.",
                       MakeTraceSourceAccessor (&MpQuicSubFlow::lastMeasuredRtt),
                       "ns3::Time::TracedValueCallback") // 追踪时间值, 回调类型是ns3::...::TracedValueCallback
        ;
      return tid;
}

/**
 * zhiy zeng: MpQuicSubFlow 类的默认构造函数, 通常在创建一个新的子流对象时调用
 * 初始化MPQUIC子流的一些基本属性（各种成员变量）, 包括
    * 地址与端口：源/目标地址和端口
    * 拥塞控制：初始拥塞窗口、慢启动阈值、状态
    * RTT 测量：RTT 估算器、最大 RTT
    * 包管理：发送序号、接收序号、未确认包历史
    * 流控与ACK：待发送 ACK 数量、是否排队发送
    * 套接字状态：创建并初始化 TCB（Transmission Control Block）
    * 事件监听：可选地绑定状态变更的回调（当前被注释）
 */
MpQuicSubFlow::MpQuicSubFlow() 
    // 成员初始化列表
    : routeId (0), // 路由ID（路径ID）, 初始化值为0
      sAddr (Ipv4Address::GetZero ()), sPort (0), // 源地址和端口, 初始化为0地址和0端口
      dAddr (Ipv4Address::GetZero ()), dPort (0), // 目标地址和端口, 初始化为0地址和0端口
      m_lastMaxData(0), // 上一次发送的MaxData帧, 默认值为0
      m_maxDataInterval(10) // MaxData帧的间隔, 默认值为10
{
    // 构造函数体内的初始化逻辑

    m_bandwidth   = 12500*0.5; // 路径带宽?

    // 初始化拥塞窗口大小, 设置为 5840 字节，即大约 4 个 MSS（1460 × 4 = 5840）
    m_cWnd        = 5840; // congestion window is initialized to one segment
    // m_Bmin = 0;
    m_segmentSize = 1460; // 报文段大小MSS, Bytes
    // 慢启动阈值, 初始化为 25000 字节, 即大约 20 个 MSS, 影响进入拥塞避免阶段的时间点
    m_ssThresh    = 25000;              // initial value for a Quic connexion
    largestRtt = Seconds(0); // 最大RTT, 初始化为0, 记录路径RTT的最大值
    m_rtt = new RttMeanDeviation (); // 指向一个 RttMeanDeviation 对象，用于测量 
    // RTT 的平均值和标准差，常用于 TCP-like 的 RTT 估计机制

    m_nextPktNum = SequenceNumber32(0); // 下一个发送的包序号, 初始化为0
    m_receivedSeqNumbers = std::vector<SequenceNumber32> (); // 接收的包序号, 初始化为空
    // 记录已接收的数据包序号，用于检测丢失
    m_unackedPackets = std::vector<MpRttHistory> (); // 未确认包及其发送时间, 初始化为空, 
    // 用于 RTT 测量和重传

    m_lost1 = 0; // 记录上一个周期（丢包事件）确认的数据量, 初始化为0
    m_lost2 = 0; // 记录当前周期（丢包事件）已确认的数据量, 初始化为0
    m_cwndState = "Slow_Start"; // 拥塞窗口状态, 初始化为"Slow_Start", 表示处于慢启动阶段
    m_lossCwnd = 12500*0.5; // 发生丢包时的拥塞窗口大小？
    m_bwEst = 0; // 带宽估计值？初始化为0
    ackSize = 0; // 累计待发送ACK数？初始化为0
    m_numPacketsReceivedSinceLastAckSent = 0; // 自上次发送ACK以来接收的包数, 初始化为0
    m_queue_ack = false; // 是否排队发送ACK, 初始化为false
    m_receivedPacketNumbers = std::vector<SequenceNumber32> (); // 已接收包的序号, 初始化为空

    // m_tcb：指向一个 QuicSocketState 对象
    m_tcb = CreateObject<QuicSocketState> ();
    m_tcb->m_cWnd = m_tcb->m_initialCWnd; // 初始化拥塞窗口大小
    m_tcb->m_ssThresh = m_tcb->m_initialSsThresh; // 初始化慢启动阈值
    m_tcb->m_pacingRate = m_tcb->m_maxPacingRate; // 初始化节流速率

    // // connect callbacks
    // bool ok;
    // ok = m_tcb->TraceConnectWithoutContext ("CongestionWindow",
    //                                         MakeCallback (&QuicSocketBase::UpdateCwnd, this));
    // NS_ASSERT_MSG (ok == true, "Failed connection to CWND trace");

    // ok = m_tcb->TraceConnectWithoutContext ("SlowStartThreshold",
    //                                         MakeCallback (&QuicSocketBase::UpdateSsThresh, this));
    // NS_ASSERT_MSG (ok == true, "Failed connection to SSTHR trace");

    // ok = m_tcb->TraceConnectWithoutContext ("CongState",
    //                                         MakeCallback (&QuicSocketBase::UpdateCongState, this));
    // NS_ASSERT_MSG (ok == true, "Failed connection to CongState trace");

    // ok = m_tcb->TraceConnectWithoutContext ("NextTxSequence",
    //                                         MakeCallback (&QuicSocketBase::UpdateNextTxSequence, this));
    // NS_ASSERT_MSG (ok == true, "Failed connection to TxSequence trace");

    // ok = m_tcb->TraceConnectWithoutContext ("HighestSequence",
    //                                         MakeCallback (&QuicSocketBase::UpdateHighTxMark, this));
    // NS_ASSERT_MSG (ok == true, "Failed connection to highest sequence trace");
}

/**
 * zhiy zeng: 初始化静态成员变量m_sst
 * Boost.Assign: 允许像函数调用一样初始化容器内容
 * 等价于：std::vector<uint32_t> v; v.push_back(50000); 
 * v.push_back(50000);
 * 所以：
 * std::vector<uint32_t> MpQuicSubFlow::m_sst = {50000, 50000};
 * 
*/ 
std::vector<uint32_t> MpQuicSubFlow::m_sst = boost::assign::list_of(50000)(50000);

/**
 * zhiy zeng: 析构函数, 用于释放资源
 */
MpQuicSubFlow::~MpQuicSubFlow()
{
    routeId     = 0;
    sAddr       = Ipv4Address::GetZero ();
    m_bandwidth   = 12500*0.5;
    m_cWnd        = 1460;
    // m_Bmin = 0;
}

/**
 * zhiy zeng: 根据传入的速率bw，计算初始的慢启动阈值
 */
void
MpQuicSubFlow::InitialRateEvent (DataRate bw) {
    //m_ssThresh = 2 * bw.GetBitRate() * m_delay / 8;
    m_sst[routeId] = 2 * bw.GetBitRate() * m_delay / 8;
}

// void
// MpQuicSubFlow::InitialRateEvent () {
//     int x = 2;
//     for (int i=x; i < 6*x+1; i = i+x) {
//         for (int j = 0;j<5;j++) {
//             if (routeId == 0) {
//                 Simulator::Schedule (Seconds (2*i-x+j*0.4-1), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
//                 // std::cout<<(2*i-x+j*0.4) <<" 0 "<<(10-j*2)*1000000*0.04/8<<"\n";
//             } else {
//                 Simulator::Schedule (Seconds (2*i-x+j*0.4-1), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
//                 // std::cout<<(2*i-x+j*0.4) <<" 1 "<<(2+j*2)*1000000*0.04/8<<"\n";
//             }  
//         }
//         if (routeId == 0) {
//             Simulator::Schedule (Seconds (2*i-1), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 0);
//             //  std::cout<< (2*i) <<" 0 "<<10*1000000*0.04/8<<"\n";
//         } else {
//             Simulator::Schedule (Seconds (2*i-1), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);
//             //  std::cout<< (2*i) <<" 1 "<<10*1000000*0.04/8<<"\n";
//         }
//     }

//     // for (int j = 0;j<5;j++) {
//     //   Simulator::Schedule (Seconds (4+j*0.8), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
//     //   Simulator::Schedule (Seconds (4+j*0.8), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
//     // }
//     // Simulator::Schedule (Seconds (8), &MpQuicSubFlow::UpdateSsh,10*1000000*m_delay/8, 0);
//     // Simulator::Schedule (Seconds (8), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);

//     // for (int j = 0;j<5;j++) {
//     //   Simulator::Schedule (Seconds (14+j*0.8), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
//     //   Simulator::Schedule (Seconds (14+j*0.8), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
//     // }
//     // Simulator::Schedule (Seconds (18), &MpQuicSubFlow::UpdateSsh,10*1000000*m_delay/8, 0);
//     // Simulator::Schedule (Seconds (18), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);
    
// }

/**
 * zhiy zeng: 模拟 QUIC 子流在不同时间点根据网络带宽和延迟变化来动态
 * 调整拥塞控制参数（如慢启动阈值） 的行为, 用于测试多路径 QUIC 在不
 * 同网络条件下的性能表现
 */
void
MpQuicSubFlow::RateChangeNotify (Time owd0, Time owd1, DataRate bw0, DataRate bw1) {
/*     int x = 2;
    for (int i=x; i < 6*x+1; i = i+x) {
        for (int j = 0;j<5;j++) {
            if (routeId == 0) {
                Simulator::Schedule (Seconds (2*i-x+j*0.4-1), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
                // std::cout<<(2*i-x+j*0.4) <<" 0 "<<(10-j*2)*1000000*0.04/8<<"\n";
            } else {
                Simulator::Schedule (Seconds (2*i-x+j*0.4-1), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
                // std::cout<<(2*i-x+j*0.4) <<" 1 "<<(2+j*2)*1000000*0.04/8<<"\n";
            }  
        }
        if (routeId == 0) {
            Simulator::Schedule (Seconds (2*i-1), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 0);
            //  std::cout<< (2*i) <<" 0 "<<10*1000000*0.04/8<<"\n";
        } else {
            Simulator::Schedule (Seconds (2*i-1), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);
            //  std::cout<< (2*i) <<" 1 "<<10*1000000*0.04/8<<"\n";
        }
    } */
    double start_time = Simulator::Now().GetSeconds();
    double delay0 = owd0.GetSeconds(); // 路径0的单向时延, seconds
    double delay1 = owd1.GetSeconds(); // 路径1的单向时延, seconds
    uint64_t bw_O_int = bw0.GetBitRate (); // 路径0的带宽, bits/s
    uint64_t bw_1_int = bw1.GetBitRate ();  // 路径1的带宽, bits/s
    for (int i = 0; i < 50; i++)
        {
            for (int j = 1; j < 5; j++)
            {
                if (routeId == 0) { // 路径0
                 //after the rtt of each path, modify the data rate
                    // 经过1个路径0的RTT后，修改路径0的慢启动阈值
                    // Simulator::Schedule: 安排一系列未来事件, 每个事件调用 UpdateSsh 方法
                    Simulator::Schedule (Seconds (start_time + (i * 4 + j) * delay0 * 2), &MpQuicSubFlow::UpdateSsh, (uint32_t)((bw_O_int - j * (bw_O_int/5))*m_delay/8), 0);
                }
                else{
                    // 经过1个路径1的RTT后，修改路径1的慢启动阈值
                    Simulator::Schedule (Seconds (start_time + (i * 4 + j) * delay1 * 2), &MpQuicSubFlow::UpdateSsh, (uint32_t)(bw_1_int / 5 * j*m_delay/8), 1);
                    // std::cout<<"time: "<< Seconds (2*i-x+j*0.4) <<" path1 : rate "<<std::to_string(2+j*2)<<"Mbps"<<"\n";
                }
                
            }
        }

    // for (int j = 0;j<5;j++) {
    //   Simulator::Schedule (Seconds (4+j*0.8), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
    //   Simulator::Schedule (Seconds (4+j*0.8), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
    // }
    // Simulator::Schedule (Seconds (8), &MpQuicSubFlow::UpdateSsh,10*1000000*m_delay/8, 0);
    // Simulator::Schedule (Seconds (8), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);

    // for (int j = 0;j<5;j++) {
    //   Simulator::Schedule (Seconds (14+j*0.8), &MpQuicSubFlow::UpdateSsh, (10-j*2)*1000000*m_delay/8, 0);
    //   Simulator::Schedule (Seconds (14+j*0.8), &MpQuicSubFlow::UpdateSsh, (2+j*2)*1000000*m_delay/8, 1);
    // }
    // Simulator::Schedule (Seconds (18), &MpQuicSubFlow::UpdateSsh,10*1000000*m_delay/8, 0);
    // Simulator::Schedule (Seconds (18), &MpQuicSubFlow::UpdateSsh, 10*1000000*m_delay/8, 1);
    
}

/**
 * zhiy zeng: 记录尚未被确认的数据包
 * 在发送数据包后，将该数据包的序列号和
 * 发送时间记录下来，以便后续接收到 ACK 后可以计算 RTT
 */
void
MpQuicSubFlow::Add (SequenceNumber32 ack) {
    // 在类型为MpRttHistory的m_unackedPackets末尾记录刚发送数据包的序列号和发送时间
    m_unackedPackets.insert (m_unackedPackets.end(),MpRttHistory (ack, Simulator::Now ()));
    // std::cout<<"\nsize "<<m_unackedPackets.size()<<"\n";
}

/**
 * zhiy zeng: 在接收到ACK时，更新RTT测量值和最大RTT
 * ack: 接收到的ACK序列号, 表示该序号之前的所有数据包已被接收方确认
 * ackDelay: 接收端处理延迟（ACK 延迟）
 */
void
MpQuicSubFlow::UpdateRtt (SequenceNumber32 ack, Time ackDelay)
{
    Time m = Time (0.0); // 初始化测量时间
    lastMeasuredRttp = lastMeasuredRtt; // 前一个RTT测量值的备份
    if (!m_unackedPackets.empty ()) // 遍历未确认包历史, 寻找匹配的ACK
    {
        std::vector<MpRttHistory>::iterator it;
        for (it = m_unackedPackets.begin (); it != m_unackedPackets.end (); ++it)
        {
            MpRttHistory item = *it;
            if (ack >= item.seq){ // 找到匹配的ACK序列号
                // 计算RTT的粗略测量值
                m = Simulator::Now () - item.time; // Elapsed time
            }
        }
    }

     
    if (!m.IsZero ()) // 如果RTT测量值不为0, 更新RTT估计值
    {
        lastMeasuredRtt = m - ackDelay; // 计算RTT估计值, 扣除ACK处理延迟

        // m_rtt->Measurement (m);                // Log the measurement
        // lastMeasuredRtt = m_rtt->GetEstimate ();// - ackDelay;
        // if (lastMeasuredRtt.Get().GetDouble() < 0){
        //     lastMeasuredRtt = m_rtt->GetEstimate ();
        // }
    }
    
    // std::cout<<Simulator::Now ().GetSeconds ()<<"flowid "<<routeId<<" seq "<<ack<<" measure: "<<lastMeasuredRtt.Get()<<"\n";
    m_rttTrace = lastMeasuredRtt; // 触发RTT追踪源, 记录RTT估计值
    if (lastMeasuredRtt>largestRtt){ // 更新最大RTT
        largestRtt = lastMeasuredRtt;
    }

    // std::cout<<Simulator::Now ().GetSeconds ()<<"flowid "<<routeId<<" seq "<<ack<<" measure: "<<m <<" measure: "<<lastMeasuredRtt <<"\n";
   
}

/**
 * zhiy zeng: 在接收到 ACK 时更新 拥塞窗口
 * 根据当前子流使用的拥塞控制算法（如 OLIA、MM-QUIC 等），
 * 动态调整拥塞窗口大小，从而控制发送速率
 * alpha: 拥塞控制因子
 * sum_rate: 当前路径的总速率
 * max_rate: 所有路径的最大速率
 * newAcks: 新确认的包列表
 * ackedBytes: 已确认的数据量
 */
void
MpQuicSubFlow::CwndOnAckReceived(double alpha, double sum_rate, double max_rate, std::vector<Ptr<QuicSocketTxItem> > newAcks, uint32_t ackedBytes)
{
    if (newAcks.size() == 0){
        return;
    }
    ackSize = newAcks.size(); // 本次确认的数据包数
    m_lost2 += ackedBytes; // 记录本周期（丢包事件）已确认的数据量
    m_throughput = ackSize*1460; // 计算吞吐量, 传输的字节数
    m_throughputBps = m_throughput*8/lastMeasuredRtt.Get().GetSeconds(); // 计算吞吐量, bits/s
    // std::cout<<routeId<<": "<<m_throughputBps<<"bps "<<m_throughput<<std::endl;
    if (m_ccType == "OLIA") // 如果拥塞控制算法为OLIA
    {
        UpdateCwndOlia(sum_rate,alpha,newAcks); // 更新拥塞窗口
    } else 
        UpdateCwndMmQuic(sum_rate, max_rate, newAcks); // 更新拥塞窗口
    // } else{
    //     UpdateCwndNewReno(sum_rate, max_rate, newAcks);
    // }
}

/**
 * zhiy zeng: 基于 OLIA 拥塞控制算法 的拥塞窗口（cwnd）更新
 * sum_rate: 所有路径的总带宽估计值
 * alpha: 控制算法中加法部分的增益因子
 * newAcks: 新确认的数据包列表
 */
void
MpQuicSubFlow::UpdateCwndOlia(double sum_rate, double alpha, std::vector<Ptr<QuicSocketTxItem> > newAcks)
{
    Ptr<QuicSocketTxItem> lastAcked = newAcks.at (0); // 获取第一个确认的包
    NS_LOG_LOGIC ("Processing acknowledged packets");
   
    // 遍历新确认的数据包, 从后往前
    for (auto it = newAcks.rbegin (); it != newAcks.rend (); ++it)
    {
        if (m_cWnd.Get() <= 4*m_segmentSize)
        { // 如果拥塞窗口小于等于4个MSS, 处于慢启动阶段
            m_cwndState = "Slow_Start";
        } 
        if (m_cWnd.Get() >= 16*m_segmentSize)
        { // 如果拥塞窗口大于等于16个MSS, 处于拥塞避免阶段
            // std::cout<<"Congestion sst"<<m_ssThresh<<" cwnd "<<m_cWnd<<"\n";
            m_cwndState = "Congestion_Avoidance";
        } 

        if ((*it)->m_acked) // 如果确认的包
        {
            if (m_cwndState == "Slow_Start") // 慢启动阶段
            {   
                // 每收到1个ACK, 拥塞窗口增加1个MSS
                m_cWnd = m_cWnd + m_segmentSize;
            } 
            else 
            {
                // 拥塞避免阶段: OLIA算法
                double increase = (m_cWnd/1460/pow(lastMeasuredRtt.Get().GetSeconds(),2))/pow(sum_rate,2)+alpha/(m_cWnd/1460);
                m_cWnd = m_cWnd + fabs(increase)*m_segmentSize;
            }
        }
    }
}


/* void
MpQuicSubFlow::UpdateCwndMmQuic(double sum_rate, double max_rate, std::vector<Ptr<QuicSocketTxItem> > newAcks)
{
    if (m_cWnd.Get() > 60000)
    {
        return;
    }
    for (auto it = newAcks.rbegin (); it != newAcks.rend (); ++it)
    {
        if (m_cWnd.Get() <= 4*m_segmentSize)
        {
            m_cwndState = "Slow_Start";
        } 
        if (m_cWnd.Get() >= m_sst[routeId])
        {
            // std::cout<<Simulator::Now ()<<" path: "<<routeId<<" avod"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            m_cwndState = "Congestion_Avoidance";
        } 

        if ((*it)->m_acked)
        {
            if (m_cwndState == "Slow_Start")
            {
                m_cWnd = m_cWnd + m_segmentSize;
                std::cout<<Simulator::Now ()<<" MpQuicSubFlow::UpdateCwndNewReno path: "<<routeId<<" Slow sst"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            } 
            else 
            {
                double increase = 3*max_rate/(2.0*lastMeasuredRtt.Get().GetSeconds()*pow(sum_rate,2.5));
                m_cWnd = m_cWnd + fabs(increase)*m_segmentSize;
                std::cout<<Simulator::Now ()<<" MpQuicSubFlow::UpdateCwndNewReno path: "<<routeId<<" avod"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            }

        }
    }
}  */

/**
 * zhiy zeng: 基于 MM-QUIC 拥塞控制算法 的拥塞窗口（cwnd）更新
 * sum_rate: 所有路径的总带宽估计值
 * max_rate: 所有路径的最大带宽估计值
 * newAcks: 新确认的数据包列表
 */
void
MpQuicSubFlow::UpdateCwndMmQuic(double sum_rate, double max_rate, std::vector<Ptr<QuicSocketTxItem> > newAcks)
{
    if (m_cWnd.Get() > 60000) // 如果拥塞窗口大于60000, 不更新
    {
        return;
    }

    // 遍历已确认数据包（从后往前）
    for (auto it = newAcks.rbegin (); it != newAcks.rend (); ++it)
    {
        if (m_cWnd.Get() <= 4*m_segmentSize) // 如果拥塞窗口小于等于4个MSS
        {
            m_cwndState = "Slow_Start";
        } 
        if (m_cWnd.Get() >= m_sst[routeId]) // 如果拥塞窗口大于等于慢启动阈值
        {
            // std::cout<<Simulator::Now ()<<" path: "<<routeId<<" avod"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            m_cwndState = "Congestion_Avoidance";
        } 

        if ((*it)->m_acked)
        {
            if (m_cwndState == "Slow_Start")
            {   
                // 每收到1个ACK, 拥塞窗口增加1个MSS
                m_cWnd = m_cWnd + m_segmentSize;
                std::cout<<Simulator::Now ()<<" MpQuicSubFlow::UpdateCwndNewReno path: "<<routeId<<" Slow sst"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            } 
            else 
            {
                // 拥塞避免阶段: MM-QUIC算法
                double increase = 3*max_rate/(2.0*lastMeasuredRtt.Get().GetSeconds()*pow(sum_rate,2.5));
                m_cWnd = m_cWnd + fabs(increase)*m_segmentSize;
                std::cout<<Simulator::Now ()<<" MpQuicSubFlow::UpdateCwndNewReno path: "<<routeId<<" avod"<<m_sst[routeId]<<" cwnd "<<m_cWnd<<"\n";
            }

        }
    }
} 

/**
 * zhiy zeng: 基于 New Reno 拥塞控制算法 的拥塞窗口（cwnd）更新
 * sum_rate: 所有路径的总带宽估计值
 * max_rate: 所有路径的最大带宽估计值
 * newAcks: 新确认的数据包列表
 */
void
MpQuicSubFlow::UpdateCwndNewReno(double sum_rate, double max_rate, std::vector<Ptr<QuicSocketTxItem> > newAcks)
{
    if (m_cWnd.Get() > 60000) // 如果拥塞窗口大于60000, 不更新
    {
        return;
    }

    // 遍历已确认数据包（从后往前）
    for (auto it = newAcks.rbegin (); it != newAcks.rend (); ++it)
    {
        if (m_cWnd.Get() >= m_ssThresh) // 如果拥塞窗口大于等于慢启动阈值
        {
            m_cwndState = "Congestion_Avoidance";
        }
        else
        {
            m_cwndState = "Slow_Start";
        }

        if ((*it)->m_acked) 
        {
            if (m_cwndState == "Slow_Start") // 慢启动阶段
            {
                // 每收到1个ACK, 拥塞窗口增加1个MSS
                m_cWnd = m_cWnd + m_segmentSize; 
            } 
            else 
            {
                // 拥塞避免阶段: New Reno算法
                m_cWnd  += 1 / m_cWnd;
            }

        }
    }
} 

/**
 * zhiy zeng: 更新慢启动阈值
 * ssh: 新的慢启动阈值
 * id: 路径ID
 */
void 
MpQuicSubFlow::UpdateSsh(uint32_t ssh, int id)
{
     m_sst[id] = ssh;
    //  std::cout<<Simulator::Now ()<<"sstsst"<<ssh<<"\n"; 
}

/* void
MpQuicSubFlow::UpdateSsThresh(double snr, uint32_t ssh)
{
    // m_ssThresh = 25000;
    m_ssThresh = ssh;
   
} */

/**
 * zhiy zeng: 减少慢启动阈值(0.5倍)
 * 该函数在发生丢包时调用, 将慢启动阈值减半
 */
void
MpQuicSubFlow::ReduceSsThresh()
{
    // m_ssThresh = 25000;
    m_ssThresh = m_cWnd / 2;
   
}

/**
 * zhiy zeng: 用于在检测到数据包丢失时更新拥塞窗口（cwnd）
 * 根据当前使用的拥塞控制算法类型（如 MM-QUIC 或其他），采取
 * 不同的降窗策略，以响应网络拥塞
 */
void
MpQuicSubFlow::UpdateCwndOnPacketLost()
{
    
    m_lost1 = m_lost2; // 记录上次丢失的包数
    m_lost2 = 0; // 重置已确认的数据量

    if ((m_ccType == "MMQUIC")) // 如果拥塞控制算法为MM-QUIC
    {
        // std::cout<<" m_throughput"<<m_throughput
        //             <<" m_ssThresh"<<m_ssThresh
        //             <<" lastMeasuredRtt"<<lastMeasuredRtt.Get().GetSeconds()
                    // <<"\n";
        // 当前吞吐量低于慢启动阈值（m_sst[routeId]）且 RTT 显著增加（超过某个延迟阈值 m_delay）
        if(m_throughput < m_sst[routeId] and lastMeasuredRtt.Get().GetSeconds() > m_delay)
        {
            m_cWnd = m_cWnd/2; // 拥塞窗口减半
            m_cwndState = "Congestion_Avoidance"; // 进入拥塞避免阶段
        }
    }
    else
    {
        m_cWnd = m_cWnd/2; // 拥塞窗口减半
        ReduceSsThresh (); // 减少慢启动阈值
    }
    
    // 记录最小损失窗口, 可用于后续路径选择、公平性判断等逻辑
    m_lossCwnd = std::min(m_lossCwnd, m_cWnd.Get());
    // m_cWnd = m_cWnd/2;
    
}

/**
 * zhiy zeng: 估算当前子流的发送速率（单位：包/秒）
 */
double
MpQuicSubFlow::GetRate()
{
    if (lastMeasuredRtt.Get().GetSeconds() == 0){
        return 0;
    }
    return m_cWnd/1460/(lastMeasuredRtt.Get().GetSeconds());
} 

/**
 * zhiy zeng: 获取当前路径在丢包发生后应使用的最小拥塞窗口值
 * 下列3个值中取最小值：
    * m_lossCwnd：记录的历史最小 cwnd（丢包时的 cwnd）
    * m_cWnd.Get()：当前的拥塞窗口
    * m_sst[routeId]：该路径的慢启动阈值（Slow Start Threshold）
 */
uint32_t
MpQuicSubFlow::GetMinPrevLossCwnd()
{
    return std::min(m_lossCwnd,std::min(m_cWnd.Get(),m_sst[routeId]));
}

/**
 * zhiy zeng: 根据拥塞控制算法类型设置初始的拥塞窗口（cwnd）
 */
void
MpQuicSubFlow::SetInitialCwnd(uint32_t cwnd)
{
    if (m_ccType == "OLIA") 
    {
        m_cWnd = 2*m_segmentSize;
    }
    else{
        m_cWnd = 4*m_segmentSize;//cwnd
    }
} 


/**
 * zhiy zeng: MpQuicAddressInfo类的默认构造函数
 * 用于初始化一个表示地址信息的对象
 */
MpQuicAddressInfo::MpQuicAddressInfo()
    : addrID (0), ipv4Addr (Ipv4Address::GetZero ()), mask (Ipv4Mask::GetZero())
{
}

/**
 * zhiy zeng: MpQuicAddressInfo类的析构函数
 */
MpQuicAddressInfo::~MpQuicAddressInfo()
{
    addrID = 0;
    ipv4Addr = Ipv4Address::GetZero ();
}

/**
 * zhiy zeng: MpRttHistory类的构造函数
 * SequenceNumber32 s: 数据包序列号
 * Time t: 包发送时间
 */
//RttHistory methods
MpRttHistory::MpRttHistory (SequenceNumber32 s, Time t)
  : seq (s),
    time (t),
    retx (false)
{
}


} // namespace ns3
