/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 SIGNET Lab, Department of Information Engineering, University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Alvise De Biasio <alvise.debiasio@gmail.com>
 *          Federico Chiariotti <whatever@blbl.it>
 *          Michele Polese <michele.polese@gmail.com>
 *          Davide Marcato <davidemarcato@outlook.com>
 *          
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/quic-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/random-variable-stream.h"
#include <iostream>
#include "ns3/flow-monitor-module.h"
#include "ns3/gnuplot.h"
#include "ns3/quic-socket-base.h"

#include <stdlib.h>
#include <unistd.h>

#include <boost/assign/list_of.hpp>

#include <iostream>
#include <string>
#include <regex>

/**
 * zhiy zeng: 一个基于 NS-3 的MPQUIC（多路径 QUIC）测试程序
 * 主要用于模拟多路径传输场景下的性能评估，支持动态调整路径参数
 * （带宽、延迟、丢包率）和调度算法，验证 MPQUIC 在不同网络条
 * 件下的表现。核心功能包括：
 * - 多路径环境配置：创建两条点到点链路模拟不同路径，支持
 * 移动场景下的动态参数修改（如带宽随时间变化）。
 * - QUIC 协议栈集成：使用 NS-3 的 QUIC 模块实现 
 * echo 客户端和服务器，支持文件传输和流量统计
 * - 性能指标收集：跟踪拥塞窗口（CWND）、往返时间
 * （RTT）、吞吐量等指标，并生成可视化图表（Gnuplot）和统计报告。
 * - 可配置参数：通过命令行调整移动性、随机模式、丢包率、调度算法
 * 等，灵活支持不同实验场景。
 * 
 * 可根据移动场景及随机移动模式, 动态修改路径带宽和延迟
 * 可设置丢包率
 * 可设置调度算法
 * 仅支持2条路径
 * 但没看到链路层信息的模拟？
 * 
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("quic-tester");

/**
 * zhiy zeng: 监听 QUIC 套接字的拥塞窗口变化，将时间、新旧拥塞窗口值写入跟踪文件
 */
// connect to a number of traces
static void
CwndChange (Ptr<OutputStreamWrapper> stream, uint32_t oldCwnd, uint32_t newCwnd)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;
}

/**
 * zhiy zeng: 监听 QUIC 套接字的 RTT 变化，记录时间、新旧 RTT 值
 */
static void
RttChange (Ptr<OutputStreamWrapper> stream, Time oldRtt, Time newRtt)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldRtt.GetSeconds () << "\t" << newRtt.GetSeconds () << std::endl;
}

// static void
// Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p, const QuicHeader& q, Ptr<const QuicSocketBase> qsb)
// {
//   std::cout<<"rxrxrx"<<"\n";
//   *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << std::endl;
// }

/**
 * zhiy zeng: 配置 QUIC 协议栈的跟踪点，连接到CwndChange和RttChange回调函数
 * 生成不同路径（子流）的拥塞窗口和 RTT 跟踪文件（如QUIC-cwnd-change-p0-<id>.txt）
 * @param serverId 服务器节点 ID（用于定位跟踪路径）
 * @param pathVersion 跟踪文件的版本路径前缀（如./server）
 * @param finalPart 跟踪文件的后缀部分（如.txt）
 */
static void
Traces(uint32_t serverId, std::string pathVersion, std::string finalPart)
{
  AsciiTraceHelper asciiTraceHelper;

  std::ostringstream pathCW;
  pathCW << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/CongestionWindow";
  NS_LOG_INFO("Matches cw " << Config::LookupMatches(pathCW.str().c_str()).GetN());

  std::ostringstream path0CW;
  path0CW << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/SubflowWindow0";
  NS_LOG_INFO("Matches cw " << Config::LookupMatches(path0CW.str().c_str()).GetN());

  std::ostringstream path1CW;
  path1CW << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/SubflowWindow1";
  NS_LOG_INFO("Matches cw " << Config::LookupMatches(path1CW.str().c_str()).GetN());

  std::ostringstream fileCW;
  fileCW << pathVersion << "QUIC-cwnd-change"  << serverId << "" << finalPart;
  std::ostringstream file0CW;
  file0CW << pathVersion << "QUIC-cwnd-change-p0-"  << serverId << "" << finalPart;
  std::ostringstream file1CW;
  file1CW << pathVersion << "QUIC-cwnd-change-p1-"  << serverId << "" << finalPart;

  // std::ostringstream pathRTT;
  // pathRTT << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/0/QuicSocketBase/RTT";

  std::ostringstream path0rtt;
  path0rtt << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/RTT0";

  std::ostringstream path1rtt;
  path1rtt << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/RTT1";
  
  std::ostringstream file0rtt;
  file0rtt << pathVersion << "QUIC-rtt-change-p0-"  << serverId << "" << finalPart;
  std::ostringstream file1rtt;
  file1rtt << pathVersion << "QUIC-rtt-change-p1-"  << serverId << "" << finalPart;
  // std::ostringstream fileRTT;
  // fileRTT << pathVersion << "QUIC-rtt"  << serverId << "" << finalPart;

  // std::ostringstream pathRCWnd;
  // pathRCWnd<< "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/0/QuicSocketBase/RWND";

  // std::ostringstream fileRCWnd;
  // fileRCWnd<<pathVersion << "QUIC-rwnd-change"  << serverId << "" << finalPart;

  // std::ostringstream fileName;
  // fileName << pathVersion << "QUIC-rx-data" << serverId << "" << finalPart;

  // std::ostringstream pathRx;
  // pathRx << "/NodeList/" << serverId << "/$ns3::QuicL4Protocol/SocketList/*/QuicSocketBase/Rx";
  // NS_LOG_INFO("Matches rx " << Config::LookupMatches(pathRx.str().c_str()).GetN());

  // Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream (fileName.str ().c_str ());
  // Config::ConnectWithoutContext (pathRx.str ().c_str (), MakeBoundCallback (&Rx, stream));

  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream (fileCW.str ().c_str ());
  Config::ConnectWithoutContext (pathCW.str ().c_str (), MakeBoundCallback(&CwndChange, stream));

  Ptr<OutputStreamWrapper> stream0 = asciiTraceHelper.CreateFileStream (file0CW.str ().c_str ());
  Config::ConnectWithoutContext (path0CW.str ().c_str (), MakeBoundCallback(&CwndChange, stream0));

  Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream (file1CW.str ().c_str ());
  Config::ConnectWithoutContext (path1CW.str ().c_str (), MakeBoundCallback(&CwndChange, stream1));

  Ptr<OutputStreamWrapper> stream0rtt = asciiTraceHelper.CreateFileStream (file0rtt.str ().c_str ());
  Config::ConnectWithoutContext (path0rtt.str ().c_str (), MakeBoundCallback(&RttChange, stream0rtt));

  Ptr<OutputStreamWrapper> stream1rtt = asciiTraceHelper.CreateFileStream (file1rtt.str ().c_str ());
  Config::ConnectWithoutContext (path1rtt.str ().c_str (), MakeBoundCallback(&RttChange, stream1rtt));


  // Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream (fileRTT.str ().c_str ());
  // Config::ConnectWithoutContext (pathRTT.str ().c_str (), MakeBoundCallback(&RttChange, stream2));

  // Ptr<OutputStreamWrapper> stream4 = asciiTraceHelper.CreateFileStream (fileRCWnd.str ().c_str ());
  // Config::ConnectWithoutContextFailSafe (pathRCWnd.str ().c_str (), MakeBoundCallback(&CwndChange, stream4));



}

std::vector<uint32_t> RxBytesList = boost::assign::list_of(0)(0);

/**
 * zhiy zeng: 周期性（每秒）通过FlowMonitor收集流量统计数据（如接收字节数）
 * 计算路径吞吐量（Mbps），更新 Gnuplot 数据集用于后续绘图
 * 核心逻辑：解析流统计信息，区分不同路径（通过FlowId），累加接收字节数并计算时间间隔内的吞吐量
 */
void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon, Gnuplot2dDataset DataSet, Gnuplot2dDataset DataSet1)
	{
    // double localThrou=0;
		std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
		Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
		for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
		{
			Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
			// std::cout<<"Flow ID			: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
			// std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
			// std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
      // std::cout<<"Duration		: "<<(stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())<<std::endl;
			// std::cout<<"Last Received Packet	: "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
			// std::cout<<"total Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps"<<std::endl;
      //localThrou=(stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024);
			// updata gnuplot data
      if (stats->first == 1) {
        DataSet.Add((double)(Simulator::Now().GetSeconds()-1),(double)(stats->second.rxBytes-RxBytesList[0])*8/1024/1024);
        RxBytesList[0] = stats->second.rxBytes;
      }
      if (stats->first == 3){
        DataSet1.Add((double)(Simulator::Now().GetSeconds()-1),(double)(stats->second.rxBytes-RxBytesList[1])*8/1024/1024);
        RxBytesList[1] = stats->second.rxBytes;
      }
      
			//std::cout<<"---------------------------------------------------------------------------"<<std::endl;
		}
			Simulator::Schedule(Seconds(1),&ThroughputMonitor, fmhelper, flowMon,DataSet, DataSet1);
         //if(flowToXml)
      // {
	    //   flowMon->SerializeToXmlFile ("ThroughputMonitor.xml", true, true);
      // }

	}



/**
 * zhiy zeng: 动态修改指定路径的带宽，并更新 QUIC 客户端的路径配置
 * @param ptp 指向 NetDeviceContainer 的指针，目标路径的设备容器（点到点链路）
 * @param echoClient QuicEchoClientHelper 对象，QUIC 客户端助手，用于同步路径带宽参数
 * @param lr 新的带宽值（DataRate 类型, 如"10Mbps"）
 * @param subflowId 子流 (路径) ID（0 或 1），用于区分不同路径
 */
void
ModifyLinkRate(NetDeviceContainer *ptp, QuicEchoClientHelper echoClient, DataRate lr, uint8_t subflowId) {
    StaticCast<PointToPointNetDevice>(ptp->Get(0))->SetDataRate(lr);
    if (subflowId == 0) echoClient.SetBW0(lr);
    else if (subflowId == 1) echoClient.SetBW1(lr);
    else std::cout<<"subflowId may be wrong!!!"<<std::endl;
    
}
  
/**
 * zhiy zeng: 初始化整个仿真流程，核心步骤包括：
 * - 参数解析：通过命令行获取移动性、丢包率、调度算法等配置
 * - 节点与协议栈创建：创建两个节点，安装 QUIC 协议栈和 Internet 协议栈
 * - 路径配置：1) 创建两条点到点链路，设置初始带宽、延迟和丢包模型; 
 *            2) 分配 IPv4 地址，绑定 QUIC 接口
 * - 应用层配置：1) 在服务器节点（n2）安装 QUIC echo 服务器，监听端口 9; 
 *              2) 在客户端节点（n1）安装 QUIC echo 客户端，配置文件传输
 *                 参数（文件大小、发包间隔）
 * - 统计与跟踪配置：1) 启用FlowMonitor收集流量统计，配置 Gnuplot 生成吞吐量曲线;
 *                  2) 调用Traces函数设置拥塞窗口和 RTT 跟踪
 * - 动态参数调整：1) 在移动场景中，按预设时间间隔修改路径带宽（支持固定模式或随机模式）
 * - 仿真执行与输出：运行仿真，生成统计报告和可视化图表（PNG 格式）
 */
int
main (int argc, char *argv[])
{
  std::cout
      << "\n\n#################### SIMULATION SET-UP ####################\n\n\n";
  
  //  Ptr<RateErrorModel> em = CreateObjectWithAttributes<RateErrorModel> (
  //     "RanVar", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
  //     "ErrorRate", DoubleValue (0.0001));

  LogLevel log_precision = LOG_ALL;
  Time::SetResolution (Time::NS);
  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);

  // LogComponentEnable ("QuicSocketTxScheduler", log_precision);
/*   LogComponentEnable ("QuicEchoClientApplication", log_precision);
  LogComponentEnable ("QuicEchoServerApplication", log_precision);  
  LogComponentEnable ("QuicHeader", log_precision);
  LogComponentEnable ("QuicSocketBase", log_precision);
  LogComponentEnable ("QuicStreamBase", log_precision);
  LogComponentEnable ("QuicL4Protocol", log_precision);
 
  
  LogComponentEnable ("QuicSocketRxBuffer", log_precision);
  LogComponentEnable ("QuicSocketTxBuffer", log_precision);
  LogComponentEnable ("QuicHeader", log_precision);
  LogComponentEnable ("QuicSubheader", log_precision);

 LogComponentEnable ("QuicL5Protocol", log_precision); */

//  LogComponentEnable ("quic-tester", log_precision);
//  LogComponentEnable ("Socket", log_precision);
//  LogComponentEnable ("Application", log_precision);
//  LogComponentEnable ("Node", log_precision);
//  LogComponentEnable ("InternetStackHelper", log_precision);
//  LogComponentEnable ("QuicSocketFactory", log_precision);
//  LogComponentEnable ("ObjectFactory", log_precision);
//  LogComponentEnable ("TypeId", log_precision);
//  LogComponentEnable ("ObjectBase", log_precision);
//  LogComponentEnable ("QuicEchoHelper", log_precision);
//  LogComponentEnable ("QuicStreamRxBuffer", log_precision);
//  LogComponentEnable ("QuicStreamTxBuffer", log_precision);
//  LogComponentEnable ("Header", log_precision);
//  LogComponentEnable ("PacketMetadata", log_precision);

  bool isMob = true; // 是否启用移动性, zhiy zeng
  bool randMob = false; // 是否启用随机移动, zhiy zeng
  std::vector<std::string> rate(2); // 路径带宽向量, zhiy zeng
  std::vector<std::string> delay(2); // 路径延迟向量, zhiy zeng
  std::vector<NetDeviceContainer> netDevices(2); // 路径设备向量, zhiy zeng
  std::string dataRate0 = "10Mbps";  // 路径0带宽, zhiy zeng
  std::string delay1 = "10ms"; // 路径1初始延迟, zhiy zeng
  delay[0] = "50ms"; // 路径0延迟, zhiy zeng
  rate[1] = "50Mbps"; // 路径1带宽, zhiy zeng

  double errorRate = 0.000004; // 丢包率, zhiy zeng
  uint8_t schAlgo = 3; // 调度算法, 2, mpquic-rr, 3. MAMS, 5. LATE, zhiy zeng
  std::string maxBuffSize = "5p"; // 路由器最大缓冲区大小, zhiy zeng
  uint64_t fileSize = 5e6; // 文件大小5MB, zhiy zeng

  CommandLine cmd;
  cmd.Usage ("Simulation of bulkSend over MPQUIC.\n"); // 使用说明：模拟块传输应用, zhiy zeng
  cmd.AddValue ("isMob", "mobility scenario", isMob); // 是否启用移动场景, zhiy zeng
  cmd.AddValue ("randMob", "mobility pattern", randMob); // 是否启用随机移动, zhiy zeng
  cmd.AddValue ("errorRate", "The percentage of packets that should be lost, expressed as a double where 1 == 100%", errorRate); // 丢包率, zhiy zeng
  cmd.AddValue ("fileSize", "file size", fileSize); // 文件大小, zhiy zeng
  cmd.AddValue ("delay1", "The initial delay for path1", delay1); // 路径1初始延迟, zhiy zeng
  cmd.AddValue ("dataRate0", "The data rate for path 0", dataRate0); // 路径0带宽, zhiy zeng
  cmd.AddValue ("maxBuffSize", "max buffer size of router", maxBuffSize); // 路由器最大缓冲区大小, zhiy zeng
  cmd.AddValue ("schAlgo", "mutipath scheduler algorithm", schAlgo); // 2, mpquic-rr, 3. MAMS, 5. LATE

  cmd.Parse (argc, argv);

  rate[0] = dataRate0; // 路径0带宽, zhiy zeng
  delay[1] = delay1; // 路径1初始延迟, zhiy zeng


  // Config::SetDefault ("ns3::QuicScheduler::SchedulerType", StringValue ("rtt"));  
  // Config::SetDefault ("ns3::MpQuicSubFlow::CCType", StringValue ("OLIA"));
  Config::SetDefault ("ns3::QuicStreamBase::StreamSndBufSize",UintegerValue(10485760)); // 设置流发送缓冲区大小为10MB
  Config::SetDefault ("ns3::QuicStreamBase::StreamRcvBufSize",UintegerValue(10485760)); // 设置流接收缓冲区大小为10MB
  Config::SetDefault ("ns3::QuicSocketBase::SocketSndBufSize",UintegerValue(10485760)); // 设置套接字发送缓冲区大小为10MB
  Config::SetDefault ("ns3::QuicSocketBase::SocketRcvBufSize",UintegerValue(10485760)); // 设置套接字接收缓冲区大小为10MB

  Config::SetDefault ("ns3::MpQuicSubFlow::delay", DoubleValue (0.03)); // 设置路径延迟为0.03s
  Config::SetDefault ("ns3::DropTailQueue<Packet>::MaxSize", StringValue (maxBuffSize)); // 设置路由器最大缓冲区大小



  NodeContainer nodes;
  nodes.Create (2); // 创建两个节点, zhiy zeng
  auto n1 = nodes.Get (0); // 获取第一个节点, zhiy zeng
  auto n2 = nodes.Get (1); // 获取第二个节点, zhiy zeng

  int sf = 2; // 子流（路径）数
  Time simulationEndTime = Seconds (50); // 模拟结束时间, zhiy zeng

  int start_time = 1; // 模拟启动时间, zhiy zeng

  QuicHelper stack;
  stack.InstallQuic (nodes); // 为节点安装QUIC协议栈, zhiy zeng

  
  float delayInt [2];
  for (int i = 0; i < delay.size(); i++) // 遍历延迟向量, zhiy zeng
    {
      std::stringstream ss(delay[i]);
      for(int j = 0; ss >> j; ) // 从 std::stringstream 中提取一个整数值到变量 j
        {
          delayInt [i] = (float) j / 1000; // 将延迟值转换为秒
        }
    }

  float bwInt [2];
  for (int i = 0; i < rate.size(); i++) // 遍历带宽向量, zhiy zeng
    {
      std::stringstream ss(rate[i]);
      for(int j = 0; ss >> j; )  // 从 std::stringstream 中提取一个整数值到变量 j
        {
          bwInt [i] = (float) j; // 将带宽值转换为浮点数
        }
    }


  // Configure the error model
  // Here we use RateErrorModel with packet error rate
  /*  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  uv->SetStream (50);
  RateErrorModel error_model;
  error_model.SetRandomVariable (uv);
  error_model.SetUnit (RateErrorModel::ERROR_UNIT_PACKET);
  error_model.SetRate (errorRate); */


  std::vector<Ipv4InterfaceContainer> ipv4Ints;

  //set drop rate

  // double errorRate = 0.1;

    Ptr<RateErrorModel> em1 = // 丢包模型
    CreateObjectWithAttributes<RateErrorModel> ("RanVar", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"), "ErrorRate", DoubleValue (errorRate));

  for(int i=0; i < sf; i++) // sf为子流（路径）数
  {
      // Creation of the point to point link between hots
      PointToPointHelper p2plink;
      p2plink.SetDeviceAttribute ("DataRate", StringValue(rate[i])); // 设置路径带宽
      p2plink.SetChannelAttribute("Delay", StringValue(delay[i])); // 设置路径延迟
     // p2plink.SetDeviceAttribute ("ReceiveErrorModel", PointerValue (&error_model));

      // NetDeviceContainer netDevices;
      netDevices[i] = p2plink.Install(nodes); // 安装点对点设备, zhiy zeng

      netDevices[i].Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em1)); // 设置接收错误模型, 即丢包模型

      std::cout<<"netdevice 0 "<<netDevices[i].Get(0)
              <<"netdevice 1 "<<netDevices[i].Get(1)
              <<"\n";
      
      // Attribution of the IP addresses, 设置IPv4地址, zhiy zeng
      std::stringstream netAddr;
      netAddr << "10.1." << (i+1) << ".0";
      std::string str = netAddr.str();

      Ipv4AddressHelper ipv4addr;
      ipv4addr.SetBase(str.c_str(), "255.255.255.0");
      Ipv4InterfaceContainer interface = ipv4addr.Assign(netDevices[i]);
      ipv4Ints.insert(ipv4Ints.end(), interface);

      p2plink.EnablePcap ("prueba" , nodes, true); // 启用pcap捕获, zhiy zeng
  }

    for (auto ipaddr:ipv4Ints)
  {
      std::cout<<"ipaddr0: "<<ipaddr.GetAddress(0)<<" ipaddr1: "<<ipaddr.GetAddress(1);
  }
  

  uint8_t dlPort = 9;

  QuicEchoServerHelper echoServer (dlPort); // 新建一个echo服务器对象，并设置服务器监听端口, zhiy zeng

  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1)); // 在节点n2上安装echo服务器, zhiy zeng
  serverApps.Start (Seconds (0.0)); // 设置echo服务器的开启时间为0.0, zhiy zeng
  serverApps.Stop (simulationEndTime); // 设置echo服务器的停止时间为simulationEndTime, zhiy zeng

  //QuicEchoClientHelper echoClient (ground_station_interfaces[1].GetAddress(0), 9);
  //for our multipath scenario, there are 4 interfaces in total, [0],[1] are for gs1; [2],[3] are for gs2 
  QuicEchoClientHelper echoClient (ipv4Ints[0].GetAddress (1), dlPort); // 新建一个echo客户端，并设置其IP及连接端口, zhiy zeng
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1)); // 设置最大发包数? zhiy zeng
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (0.01))); // 设置发包间隔, zhiy zeng
  echoClient.SetAttribute ("PacketSize", UintegerValue(1460)); // 设置包大小？zhiy zeng
  echoClient.SetIniRTT0 (Seconds (delayInt [0])); // 设置路径0的初始延迟, zhiy zeng
  echoClient.SetIniRTT1 (Seconds (delayInt [1])); // 设置路径1的初始延迟, zhiy zeng
  echoClient.SetER (errorRate); // 设置丢包率, zhiy zeng
  echoClient.SetBW0 (DataRate (rate[0])); // 设置路径0的带宽, zhiy zeng
  echoClient.SetBW1 (DataRate (rate[1])); // 设置路径1的带宽, zhiy zeng
  echoClient.SetScheAlgo (schAlgo); // 设置调度算法, zhiy zeng
  echoClient.WithMobility (isMob); // 设置是否启用移动性, zhiy zeng

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0)); // 在节点n1上安装echo客户端, zhiy zeng
  //echoClient.SetFill (clientApps.Get (0),"Hello World");
  echoClient.SetFill (clientApps.Get (0),100,fileSize); // 设置填充数据, zhiy zeng
  clientApps.Start (Seconds (start_time)); // 设置echo客户端的开启时间, zhiy zeng
  clientApps.Stop (simulationEndTime); // 设置echo客户端的停止时间, zhiy zeng



//   uint16_t port = 9;  // well-known echo port number

//   uint32_t maxBytes = 5000;
//   BulkSendHelper source ("ns3::QuicSocketFactory",
//                          InetSocketAddress (ipv4Ints[0].GetAddress (1), port));
//   // Set the amount of data to send in bytes.  Zero is unlimited.
//   source.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
//   ApplicationContainer sourceApps = source.Install (nodes.Get (0));
//   sourceApps.Start (Seconds (start_time));
//   sourceApps.Stop (simulationEndTime);

// //
// // Create a PacketSinkApplication and install it on node 1
// //
//   PacketSinkHelper sink ("ns3::QuicSocketFactory",
//                          InetSocketAddress (Ipv4Address::GetAny (), port));
//   ApplicationContainer sinkApps = sink.Install (nodes.Get (1));
//   sinkApps.Start (Seconds (0.0));
//   sinkApps.Stop (simulationEndTime);


  Simulator::Schedule (Seconds (start_time+0.0000001), &Traces, n2->GetId(),
        "./server", ".txt"); // 记录服务器端的跟踪信息, zhiy zeng
  Simulator::Schedule (Seconds (start_time+0.0000001), &Traces, n1->GetId(),
        "./client", ".txt"); // 记录客户端的跟踪信息, zhiy zeng

  Packet::EnablePrinting (); // 启用数据包打印, zhiy zeng
  Packet::EnableChecking (); // 启用数据包检查, zhiy zeng


    std::string fileNameWithNoExtension = "FlowVSThroughput_";
    std::string graphicsFileName        = fileNameWithNoExtension + ".png";
    std::string plotFileName            = fileNameWithNoExtension + ".plt";
    std::string plotTitle               = "Throughput vs Time";
    std::string dataTitle               = "path 0";
    std::string dataTitle1               = "path 1";

    // Instantiate the plot and set its title.
    Gnuplot gnuplot (graphicsFileName);
    gnuplot.SetTitle (plotTitle);

    // Make the graphics file, which the plot file will be when it
    // is used with Gnuplot, be a PNG file.
    gnuplot.SetTerminal ("png");

    // Set the labels for each axis.
    gnuplot.SetLegend ("Time(s)", "Throughput(Mbps)");

     
   Gnuplot2dDataset dataset;
   dataset.SetTitle (dataTitle);
   dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
   Gnuplot2dDataset dataset1;
   dataset1.SetTitle (dataTitle1);
   dataset1.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  //flowMonitor declaration
  // call the flow monitor function
  // std::ofstream outfile1;
  // outfile1.open("flowmonitor.txt");
  ThroughputMonitor(&flowmon, monitor, dataset, dataset1); 
  // outfile1.close();

  if (isMob) // 如果处于移动场景, zhiy zeng
    {
      for (int i = 0; i < 50; i++)
        {
          for (int j = 1; j < 5; j++)
            {
              //after the rtt of each path, modify the data rate, 间隔一个路径的rtt，修改路径带宽
              Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[0] * 2), &ModifyLinkRate, &netDevices[0], echoClient, DataRate(std::to_string(bwInt[0]-j*(bwInt[0]/5))+"Mbps"), 0);
              // std::cout<<"time: "<< start_time + (i * 4 + j) * delayInt[0] * 2 <<" path0 : rate "<<bwInt[0]-j*(bwInt[0]/5)<<"Mbps"<<"\n";
              Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[1] * 2), &ModifyLinkRate, &netDevices[1], echoClient, DataRate(std::to_string(bwInt[0]/5*(j+1))+"Mbps"), 1);
              // std::cout<<"time: "<< start_time + (i * 4 + j) * delayInt[1] * 2 <<" path1 : rate "<<bwInt[0]/5*(j+1)<<"Mbps"<<"\n";
            }
        }
    }


      if (isMob) // 如果处于移动场景, zhiy zeng
      { 
        for (int i = 0; i < 50; i++)
          {
            for (int j = 1; j < 5; j++)
              {
                if (randMob) // 如果采用随机移动模式, zhiy zeng
                {
                  // 采用均匀分布产生随机带宽
                  Ptr<ns3::NormalRandomVariable> rate = CreateObject<NormalRandomVariable> ();
                  rate->SetAttribute ("Mean", DoubleValue (bwInt[0]));
                  rate->SetAttribute ("Variance", DoubleValue (bwInt[0]/10));
                  Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[0] * 2), &ModifyLinkRate, &netDevices[0], echoClient, DataRate(std::to_string(rate->GetValue())+"Mbps"), 0);

                  rate->SetAttribute ("Mean", DoubleValue (bwInt[1]));
                  rate->SetAttribute ("Variance", DoubleValue (bwInt[1]/10));
                  Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[1] * 2), &ModifyLinkRate, &netDevices[1], echoClient, DataRate(std::to_string(rate->GetValue())+"Mbps"), 1);
                }
                else{
                  //after the rtt of each path, modify the data rate
                  Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[0] * 2), &ModifyLinkRate, &netDevices[0], echoClient, DataRate(std::to_string(bwInt[0]-j*(bwInt[0]/5))+"Mbps"), 0);
                  // std::cout<<"time: "<< start_time + (i * 4 + j) * delayInt[0] * 2 <<" path0 : rate "<<bwInt[0]-j*(bwInt[0]/5)<<"Mbps"<<"\n";
                  Simulator::Schedule (Seconds (start_time + (i * 4 + j + 1) * delayInt[1] * 2), &ModifyLinkRate, &netDevices[1], echoClient, DataRate(std::to_string(bwInt[0]/5*(j+1))+"Mbps"), 1);
                  // std::cout<<"time: "<< start_time + (i * 4 + j) * delayInt[1] * 2 <<" path1 : rate "<<bwInt[0]/5*(j+1)<<"Mbps"<<"\n";
                }
                
              }
          }
      }
  


  Simulator::Stop (simulationEndTime);
  std::cout << "\n\n#################### STARTING RUN ####################\n\n";
  Simulator::Run ();

  //Gnuplot ...continued
  gnuplot.AddDataset (dataset);
  gnuplot.AddDataset (dataset1);

  dataset.Add(0,0);
  dataset.Add(0,0);
  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());
  // Write the plot file.
  gnuplot.GenerateOutput (plotFile);
  // Close the plot file.
  plotFile.close ();

  flowmon.SerializeToXmlFile("flow", false, false);

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();

  std::ofstream outfile;
  outfile.open("wmp"+std::to_string(simulationEndTime.GetSeconds())+".txt");
/*   for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      outfile << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
      outfile << "  Tx Packets: " << i->second.txPackets << "\n";
      outfile << "  Tx Bytes:   " << i->second.txBytes << "\n";
      outfile << "  TxOffered:  " << i->second.txBytes * 8.0 / simulationEndTime.GetSeconds () / 1000 / 1000  << " Mbps\n";
      outfile << "  Rx Packets: " << i->second.rxPackets << "\n";
      outfile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
      outfile << "  Throughput: " << i->second.rxBytes * 8.0 / simulationEndTime.GetSeconds () / 1000 / 1000  << " Mbps\n";
      outfile << "  Tx time: " << i->second.timeLastTxPacket - i->second.timeFirstTxPacket<<"\n";
      outfile << "  Rx time: " << i->second.timeLastRxPacket - i->second.timeFirstRxPacket<<"\n";
      outfile << "delay sum" << i->second.delaySum<<"\n";
      std::cout  <<  "  Tx Bytes:   " << i->second.txBytes << "\n";
      std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / simulationEndTime.GetSeconds () / 1000 / 1000  << " Mbps\n";    
      std::cout <<  "  Tx time: " << (i->second.timeLastTxPacket - i->second.timeFirstTxPacket).GetSeconds()<<"\n";  
    
    
    } */

  outfile.close();
  // std::cout
  //     << "\n\n#################### RUN FINISHED ####################\n\n\n";
  Simulator::Destroy ();

  // Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
  // std::cout << "Total Bytes Received: " << sink1->GetTotalRx () << std::endl;
  // std::cout
  //     << "\n\n#################### SIMULATION END ####################\n\n\n";
  return 0;
}

