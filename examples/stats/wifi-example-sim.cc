/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Authors: Joe Kopena <tjkopena@cs.drexel.edu>
 *
 * This program conducts a simple experiment: It places two nodes at a
 * parameterized distance apart.  One node generates packets and the
 * other node receives.  The stat framework collects data on packet
 * loss.  Outside of this program, a control script uses that data to
 * produce graphs presenting performance at the varying distances.
 * This isn't a typical simulation but is a common "experiment"
 * performed in real life and serves as an accessible exemplar for the
 * stat framework.  It also gives some intuition on the behavior and
 * basic reasonability of the NS-3 WiFi models.
 *
 * Applications used by this program are in test02-apps.h and
 * test02-apps.cc, which should be in the same place as this file.
 * 
 */


/**
 * zhiy zeng: 用于测试 WiFi 信号在不同距离下的传输性能。它创建了两个节点，
 * 一个发送数据包，另一个接收，通过改变它们之间的距离来收集不同距离下的网络
 * 性能数据（如丢包率、延迟），并使用 NS-3 的统计框架将结果输出为指定格式
 * （OMNeT 或 SQLite）。这些数据可用于分析 WiFi 信号强度随距离衰减的特性，
 * 验证 NS-3 WiFi 模型的合理性。
 */

#include <ctime>
#include <sstream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/stats-module.h"
#include "ns3/yans-wifi-helper.h"
#include "wifi-example-apps.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("WiFiDistanceExperiment");

/**
 * zhiy zeng: 作为 WiFi MAC 层发送数据包的回调函数，每当发送一个帧时，更新计数器
 */
void TxCallback (Ptr<CounterCalculator<uint32_t> > datac,
                 std::string path, Ptr<const Packet> packet) {
  NS_LOG_INFO ("Sent frame counted in " <<
               datac->GetKey ());
  datac->Update ();
  // end TxCallback
}




//----------------------------------------------------------------------
//-- main
//----------------------------------------------
/**
 * zhiy zeng: 主函数，整个实验的入口点，负责设置参数、创建网络拓扑、配置统计收集器并运行仿真
 */
int main (int argc, char *argv[]) {

  double distance = 50.0;
  string format ("omnet");

  string experiment ("wifi-distance-test");
  string strategy ("wifi-default");
  string input;
  string runID;

  {
    stringstream sstr;
    sstr << "run-" << time (NULL);
    runID = sstr.str ();
  }

  // Set up command line parameters used to control the experiment.
  CommandLine cmd (__FILE__);
  cmd.AddValue ("distance", "Distance apart to place nodes (in meters).",
                distance);
  cmd.AddValue ("format", "Format to use for data output.",
                format);
  cmd.AddValue ("experiment", "Identifier for experiment.",
                experiment);
  cmd.AddValue ("strategy", "Identifier for strategy.",
                strategy);
  cmd.AddValue ("run", "Identifier for run.",
                runID);
  cmd.Parse (argc, argv);

  if (format != "omnet" && format != "db") {
      NS_LOG_ERROR ("Unknown output format '" << format << "'");
      return -1;
    }

  #ifndef STATS_HAS_SQLITE3
  if (format == "db") {
      NS_LOG_ERROR ("sqlite support not compiled in.");
      return -1;
    }
  #endif

  {
    stringstream sstr ("");
    sstr << distance;
    input = sstr.str ();
  }




  //------------------------------------------------------------
  //-- Create nodes and network stacks
  //--------------------------------------------
  NS_LOG_INFO ("Creating nodes.");
  NodeContainer nodes;
  nodes.Create (2);

  NS_LOG_INFO ("Installing WiFi and Internet stack.");
  WifiHelper wifi;
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  NetDeviceContainer nodeDevices = wifi.Install (wifiPhy, wifiMac, nodes);

  InternetStackHelper internet;
  internet.Install (nodes);
  Ipv4AddressHelper ipAddrs;
  ipAddrs.SetBase ("192.168.0.0", "255.255.255.0");
  ipAddrs.Assign (nodeDevices);




  //------------------------------------------------------------
  //-- Setup physical layout
  //--------------------------------------------
  NS_LOG_INFO ("Installing static mobility; distance " << distance << " .");
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc =
    CreateObject<ListPositionAllocator>();
  ///////////////////////////////////////////////////
  // 设置节点位置，第一个节点位于原点 (0,0,0)，第二个节点沿
  // Y 轴方向与第一个节点相距distance米
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (0.0, distance, 0.0));
  ///////////////////////////////////////////////////
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (nodes);




  //------------------------------------------------------------
  //-- Create a custom traffic source and sink
  //--------------------------------------------
  NS_LOG_INFO ("Create traffic source & sink.");
  Ptr<Node> appSource = NodeList::GetNode (0);
  Ptr<Sender> sender = CreateObject<Sender>();
  appSource->AddApplication (sender); // 节点0安装发送器应用
  sender->SetStartTime (Seconds (1));

  Ptr<Node> appSink = NodeList::GetNode (1);
  Ptr<Receiver> receiver = CreateObject<Receiver>();
  appSink->AddApplication (receiver); // 节点1安装接收器应用
  receiver->SetStartTime (Seconds (0));

  Config::Set ("/NodeList/*/ApplicationList/*/$Sender/Destination",
               Ipv4AddressValue ("192.168.0.2")); // 设置发送器的目标地址为节点1的IP地址




  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  // Create a DataCollector object to hold information about this run.
  DataCollector data;
  data.DescribeRun (experiment,
                    strategy,
                    input,
                    runID);

  // Add any information we wish to record about this run.
  data.AddMetadata ("author", "tjkopena");


  // Create a counter to track how many frames are generated.  Updates
  // are triggered by the trace signal generated by the WiFi MAC model
  // object.  Here we connect the counter to the signal via the simple
  // TxCallback() glue function defined above.
  Ptr<CounterCalculator<uint32_t> > totalTx =
    CreateObject<CounterCalculator<uint32_t> >();
  totalTx->SetKey ("wifi-tx-frames"); // WiFi MAC 层发送的帧数
  totalTx->SetContext ("node[0]");
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                   MakeBoundCallback (&TxCallback, totalTx));
  data.AddDataCalculator (totalTx);

  // This is similar, but creates a counter to track how many frames
  // are received.  Instead of our own glue function, this uses a
  // method of an adapter class to connect a counter directly to the
  // trace signal generated by the WiFi MAC.
  Ptr<PacketCounterCalculator> totalRx =
    CreateObject<PacketCounterCalculator>();
  totalRx->SetKey ("wifi-rx-frames"); // WiFi MAC 层接收的帧数
  totalRx->SetContext ("node[1]");
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&PacketCounterCalculator::PacketUpdate,
                                 totalRx));
  data.AddDataCalculator (totalRx);




  // This counter tracks how many packets---as opposed to frames---are
  // generated.  This is connected directly to a trace signal provided
  // by our Sender class.
  Ptr<PacketCounterCalculator> appTx =
    CreateObject<PacketCounterCalculator>();
  appTx->SetKey ("sender-tx-packets"); // 发送器应用发送的包数
  appTx->SetContext ("node[0]");
  Config::Connect ("/NodeList/0/ApplicationList/*/$Sender/Tx",
                   MakeCallback (&PacketCounterCalculator::PacketUpdate,
                                 appTx));
  data.AddDataCalculator (appTx);

  // Here a counter for received packets is directly manipulated by
  // one of the custom objects in our simulation, the Receiver
  // Application.  The Receiver object is given a pointer to the
  // counter and calls its Update() method whenever a packet arrives.
  Ptr<CounterCalculator<> > appRx =
    CreateObject<CounterCalculator<> >();
  appRx->SetKey ("receiver-rx-packets"); // 接收器应用接收的包数
  appRx->SetContext ("node[1]");
  receiver->SetCounter (appRx);
  data.AddDataCalculator (appRx);




  /**
   * Just to show this is here...
   Ptr<MinMaxAvgTotalCalculator<uint32_t> > test = 
   CreateObject<MinMaxAvgTotalCalculator<uint32_t> >();
   test->SetKey("test-dc");
   data.AddDataCalculator(test);

   test->Update(4);
   test->Update(8);
   test->Update(24);
   test->Update(12);
  **/

  // This DataCalculator connects directly to the transmit trace
  // provided by our Sender Application.  It records some basic
  // statistics about the sizes of the packets received (min, max,
  // avg, total # bytes), although in this scenaro they're fixed.
  Ptr<PacketSizeMinMaxAvgTotalCalculator> appTxPkts =
    CreateObject<PacketSizeMinMaxAvgTotalCalculator>();
  appTxPkts->SetKey ("tx-pkt-size"); // 发送器应用发送的包大小统计
  appTxPkts->SetContext ("node[0]");
  Config::Connect ("/NodeList/0/ApplicationList/*/$Sender/Tx",
                   MakeCallback
                     (&PacketSizeMinMaxAvgTotalCalculator::PacketUpdate,
                     appTxPkts));
  data.AddDataCalculator (appTxPkts);


  // Here we directly manipulate another DataCollector tracking min,
  // max, total, and average propagation delays.  Check out the Sender
  // and Receiver classes to see how packets are tagged with
  // timestamps to do this.
  Ptr<TimeMinMaxAvgTotalCalculator> delayStat =
    CreateObject<TimeMinMaxAvgTotalCalculator>();
  delayStat->SetKey ("delay"); // 传输延迟统计
  delayStat->SetContext (".");
  receiver->SetDelayTracker (delayStat);
  data.AddDataCalculator (delayStat);




  //------------------------------------------------------------
  //-- Run the simulation
  //--------------------------------------------
  NS_LOG_INFO ("Run Simulation.");
  Simulator::Run ();




  //------------------------------------------------------------
  //-- Generate statistics output.
  //--------------------------------------------

  // Pick an output writer based in the requested format.
  Ptr<DataOutputInterface> output = 0;
  if (format == "omnet") {
      NS_LOG_INFO ("Creating omnet formatted data output.");
      output = CreateObject<OmnetDataOutput>();
    } else if (format == "db") {
    #ifdef STATS_HAS_SQLITE3
      NS_LOG_INFO ("Creating sqlite formatted data output.");
      output = CreateObject<SqliteDataOutput>();
    #endif
    } else {
      NS_LOG_ERROR ("Unknown output format " << format);
    }

  // Finally, have that writer interrogate the DataCollector and save
  // the results.
  if (output != 0)
    output->Output (data);

  // Free any memory here at the end of this example.
  Simulator::Destroy ();

  // end main
}
