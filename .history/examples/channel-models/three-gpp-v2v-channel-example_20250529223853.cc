/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020, University of Padova, Dep. of Information Engineering, SIGNET lab
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
 */

/**
* This is an example on how to configure the channel model classes to simulate
* a vehicular environment.
* The channel condition is determined using the model specified in [1], Table 6.2-1.
* The pathloss is determined using the model specified in [1], Table 6.2.1-1.
* The model for the fast fading is the one described in 3GPP TR 38.901 v15.0.0,
* the model parameters are those specified in [1], Table 6.2.3-1.
*
* This example generates the output file 'example-output.txt'. Each row of the
* file is organized as follows:
* Time[s] TxPosX[m] TxPosY[m] RxPosX[m] RxPosY[m] ChannelState SNR[dB] Pathloss[dB]
* We also provide a bash script which reads the output file and generates two
* figures:
* (i) map.gif, a GIF representing the simulation scenario and vehicle mobility;
* (ii) snr.png, which represents the behavior of the SNR.
*
* [1] 3GPP TR 37.885, v15.3.0
*/

/**
 * zhiy zeng
 * 用于模拟车联网（V2V）通信场景下的信道模型的示例代码，基于 3GPP 标准实现了以下核心功能：
 * 配置车联网传播环境（城市或高速公路场景），包括建筑物布局、车辆移动轨迹。
 * 模拟 3GPP 定义的信道特性：路径损耗、快衰落、波束成形（DFT 波束赋形）和信道状态（LOS/NLOS）。
 * 输出仿真数据：记录时间、节点位置、信道状态、SNR 和路径损耗，用于后续分析和可视化（如生成 GIF 和 SNR 曲线）。
 */

#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <fstream>
#include "ns3/three-gpp-antenna-array-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/three-gpp-channel-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ThreeGppV2vChannelExample");

static Ptr<ThreeGppPropagationLossModel> m_propagationLossModel; //!< the PropagationLossModel object
static Ptr<ThreeGppSpectrumPropagationLossModel> m_spectrumLossModel; //!< the SpectrumPropagationLossModel object
static Ptr<ChannelConditionModel> m_condModel; //!< the ChannelConditionModel object

/**
 * Perform the beamforming using the DFT beamforming method
 * \param thisDevice the device performing the beamforming
 * \param thisAntenna the antenna object associated to thisDevice
 * \param otherDevice the device towards which point the beam
 */

/**
 * zhiy zeng: 实现 DFT 波束赋形，根据收发节点位置计算天线权重，使信号能量集中于目标方向
 * @param thisDevice 执行波束赋形的设备, 即发射设备（含天线模型）
 * @param thisAntenna 设备关联的天线模型, 发射天线阵列模型
 * @param otherDevice 目标设备, 即接收设备（目标节点）
 * 
 * 核心逻辑
 * 1. 获取收发节点坐标，计算方位角（phi）和仰角（theta）
 * 2. 根据天线阵元位置和角度，生成相位偏移量，计算各阵元的加权系数（复数形式）
 * 3. 通过SetBeamformingVector设置天线权重，实现波束指向优化
 */
static void
DoBeamforming (Ptr<NetDevice> thisDevice, Ptr<ThreeGppAntennaArrayModel> thisAntenna, Ptr<NetDevice> otherDevice)
{
  ThreeGppAntennaArrayModel::ComplexVector antennaWeights;

  // retrieve the position of the two devices
  Vector aPos = thisDevice->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();
  Vector bPos = otherDevice->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();

  // compute the azimuth and the elevation angles
  Angles completeAngle (bPos,aPos);

  double hAngleRadian = fmod (completeAngle.phi, 2.0 * M_PI); // the azimuth angle
  if (hAngleRadian < 0)
    {
      hAngleRadian += 2.0 * M_PI;
    }
  double vAngleRadian = completeAngle.theta; // the elevation angle

  // retrieve the number of antenna elements
  int totNoArrayElements = thisAntenna->GetNumberOfElements ();

  // the total power is divided equally among the antenna elements
  double power = 1 / sqrt (totNoArrayElements);

  // compute the antenna weights
  for (int ind = 0; ind < totNoArrayElements; ind++)
    {
      Vector loc = thisAntenna->GetElementLocation (ind);
      double phase = -2 * M_PI * (sin (vAngleRadian) * cos (hAngleRadian) * loc.x
                                  + sin (vAngleRadian) * sin (hAngleRadian) * loc.y
                                  + cos (vAngleRadian) * loc.z);
      antennaWeights.push_back (exp (std::complex<double> (0, phase)) * power);
    }

  // store the antenna weights
  thisAntenna->SetBeamformingVector (antennaWeights);
}

/**
 * Compute the average SNR
 * \param txMob the tx mobility model
 * \param rxMob the rx mobility model
 * \param txPsd the PSD of the transmitting signal
 * \param noiseFigure the noise figure in dB
 */
/**
 * zhiy zeng: 计算信噪比（SNR），并记录仿真数据（时间、位置、信道状态、SNR、路径损耗）
 * @param txMob 发射端的移动模型, 即发送设备的移动模型
 * @param rxMob 接收端的移动模型, 即接收设备的移动模型
 * @param txPsd 发射信号的功率谱密度（PSD）, 即发送设备发射信号的功率谱密度
 * @param noiseFigure 噪声系数（dB）, 即接收设备的噪声系数
 * 
 * 核心逻辑
 * 1. 获取信道状态：通过m_condModel判断 LOS/NLOS 条件
 * 2. 计算路径损耗：调用m_propagationLossModel计算接收功率增益（线性值），转换为路径损耗（dB）
 * 3. 应用快衰落和波束增益：通过m_spectrumLossModel模拟频率选择性衰落和天线增益
 * 4. 计算噪声功率：基于热噪声公式（kT）和噪声系数生成噪声 PSD
 * 5. 输出数据：将结果写入example-output.txt，格式为逗号分隔的数值
 */

static void
ComputeSnr (Ptr<MobilityModel> txMob, Ptr<MobilityModel> rxMob, Ptr<const SpectrumValue> txPsd, double noiseFigure)
{
  Ptr<SpectrumValue> rxPsd = txPsd->Copy ();

  // check the channel condition
  Ptr<ChannelCondition> cond = m_condModel->GetChannelCondition (txMob, rxMob);

  // apply the pathloss
  double propagationGainDb = m_propagationLossModel->CalcRxPower (0, txMob, rxMob);
  NS_LOG_DEBUG ("Pathloss " << -propagationGainDb << " dB");
  double propagationGainLinear = std::pow (10.0, (propagationGainDb) / 10.0);
  *(rxPsd) *= propagationGainLinear;

  // apply the fast fading and the beamforming gain
  rxPsd = m_spectrumLossModel->CalcRxPowerSpectralDensity (rxPsd, txMob, rxMob);
  NS_LOG_DEBUG ("Average rx power " << 10 * log10 (Sum (*rxPsd) * 180e3) << " dB");

  // create the noise psd
  // taken from lte-spectrum-value-helper
  const double kT_dBm_Hz = -174.0; // dBm/Hz
  double kT_W_Hz = std::pow (10.0, (kT_dBm_Hz - 30) / 10.0);
  double noiseFigureLinear = std::pow (10.0, noiseFigure / 10.0);
  double noisePowerSpectralDensity =  kT_W_Hz * noiseFigureLinear;
  Ptr<SpectrumValue> noisePsd = Create <SpectrumValue> (txPsd->GetSpectrumModel ());
  (*noisePsd) = noisePowerSpectralDensity;

  // compute the SNR
  NS_LOG_DEBUG ("Average SNR " << 10 * log10 (Sum (*rxPsd) / Sum (*noisePsd)) << " dB");

  // print the SNR and pathloss values in the snr-trace.txt file
  std::ofstream f;
  f.open ("example-output.txt", std::ios::out | std::ios::app);
  f << Simulator::Now ().GetSeconds () << " " // time [s]
    << txMob->GetPosition ().x << " "
    << txMob->GetPosition ().y << " "
    << rxMob->GetPosition ().x << " "
    << rxMob->GetPosition ().y << " "
    << cond->GetLosCondition () << " " // channel state
    << 10 * log10 (Sum (*rxPsd) / Sum (*noisePsd)) << " " // SNR [dB]
    << -propagationGainDb << std::endl; // pathloss [dB]
  f.close ();
}

/**
 * Generates a GNU-plottable file representig the buildings deployed in the
 * scenario
 * \param filename the name of the output file
 */

/**
 * zhiy zeng: 将场景中的建筑物布局导出为 Gnuplot 可识别的坐标文件，用于后续可视化
 * @param filename 输出文件名, 即生成的 Gnuplot 文件名
 * 
 * 核心逻辑
 * 1. 遍历所有建筑物，记录其边界坐标（矩形范围），生成 Gnuplot 的set object指令，保存至文件
 */
void
PrintGnuplottableBuildingListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  uint32_t index = 0;
  for (BuildingList::Iterator it = BuildingList::Begin (); it != BuildingList::End (); ++it)
    {
      ++index;
      Box box = (*it)->GetBoundaries ();
      outFile << "set object " << index
              << " rect from " << box.xMin  << "," << box.yMin
              << " to "   << box.xMax  << "," << box.yMax
              << std::endl;
    }
}

/**
 * zhiy zeng: 初始化仿真环境，配置参数，创建节点、天线、移动模型和信道模型，并启动仿真
 * 
 * 核心逻辑
 * 1. 参数配置：设置频率、发射功率、噪声系数、场景类型（城市或高速公路）、车辆速度、子载波间隔和资源块数量等。
 * 2. 创建节点：生成两个节点，分别作为发送端和接收端, 绑定SimpleNetDevice和ThreeGppAntennaArrayModel（2x2 天线阵列）。
 * 3. 设置移动模型：根据场景类型（城市或高速公路）创建车辆的移动轨迹，使用WaypointMobilityModel模拟车辆在道路上的移动, 
 *    使用ConstantVelocityMobilityModel定义车辆匀速直线运动（前后行驶，间距 20 米）
 * 4. 信道模型初始化: 根据场景类型选择ThreeGppV2vUrban/V2vHighwayChannelConditionModel和路径损耗模型
 *    配置ThreeGppChannelModel和ThreeGppSpectrumPropagationLossModel，关联天线和设备
 * 5. 波束成形与 PSD 设置：调用DoBeamforming初始化收发端波束方向
 *    生成资源块（RB）对应的功率谱密度，模拟多载波信号（如 5G NR）
 * 6. 仿真调度与运行：按时间步长（timeRes=10ms）调度ComputeSnr函数，周期性计算 SNR 和路径损耗
 *    初始化输出文件头，运行仿真并销毁资源
 */
int
main (int argc, char *argv[])
{
  double frequency = 28.0e9; // operating frequency in Hz
  double txPow_dbm = 30.0; // tx power in dBm
  double noiseFigure = 9.0; // noise figure in dB
  Time simTime = Seconds (40); // simulation time
  Time timeRes = MilliSeconds (10); // time resolution
  std::string scenario = "V2V-Urban"; // 3GPP propagation scenario, V2V-Urban or V2V-Highway
  double vScatt = 0; // maximum speed of the vehicles in the scenario [m/s]
  double subCarrierSpacing = 60e3; // subcarrier spacing in kHz
  uint32_t numRb = 275; // number of resource blocks

  CommandLine cmd (__FILE__);
  cmd.AddValue ("frequency", "operating frequency in Hz", frequency);
  cmd.AddValue ("txPow", "tx power in dBm", txPow_dbm);
  cmd.AddValue ("noiseFigure", "noise figure in dB", noiseFigure);
  cmd.AddValue ("scenario", "3GPP propagation scenario, V2V-Urban or V2V-Highway", scenario);
  cmd.Parse (argc, argv);

  // create the nodes
  NodeContainer nodes;
  nodes.Create (2);

  // create the tx and rx devices
  Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice> ();
  Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice> ();

  // associate the nodes and the devices
  nodes.Get (0)->AddDevice (txDev);
  txDev->SetNode (nodes.Get (0));
  nodes.Get (1)->AddDevice (rxDev);
  rxDev->SetNode (nodes.Get (1));

  // create the antenna objects and set their dimensions
  Ptr<ThreeGppAntennaArrayModel> txAntenna = CreateObjectWithAttributes<ThreeGppAntennaArrayModel> ("NumColumns", UintegerValue (2), "NumRows", UintegerValue (2), "BearingAngle", DoubleValue (-M_PI / 2));
  Ptr<ThreeGppAntennaArrayModel> rxAntenna = CreateObjectWithAttributes<ThreeGppAntennaArrayModel> ("NumColumns", UintegerValue (2), "NumRows", UintegerValue (2), "BearingAngle", DoubleValue (M_PI / 2));

  Ptr<MobilityModel> txMob;
  Ptr<MobilityModel> rxMob;
  if (scenario == "V2V-Urban")
    {
      // 3GPP defines that the maximum speed in urban scenario is 60 km/h
      vScatt = 60 / 3.6;

      // create a grid of buildings
      double buildingSizeX = 250 - 3.5 * 2 - 3; // m
      double buildingSizeY = 433 - 3.5 * 2 - 3; // m
      double streetWidth = 20; // m
      double buildingHeight = 10; // m
      uint32_t numBuildingsX = 2;
      uint32_t numBuildingsY = 2;
      double maxAxisX = (buildingSizeX + streetWidth) * numBuildingsX;
      double maxAxisY = (buildingSizeY + streetWidth) * numBuildingsY;

      std::vector<Ptr<Building> > buildingVector;
      for (uint32_t buildingIdX = 0; buildingIdX < numBuildingsX; ++buildingIdX)
        {
          for (uint32_t buildingIdY = 0; buildingIdY < numBuildingsY; ++buildingIdY)
            {
              Ptr < Building > building;
              building = CreateObject<Building> ();

              building->SetBoundaries (Box (buildingIdX * (buildingSizeX + streetWidth),
                                            buildingIdX * (buildingSizeX + streetWidth) + buildingSizeX,
                                            buildingIdY * (buildingSizeY + streetWidth),
                                            buildingIdY * (buildingSizeY + streetWidth) + buildingSizeY,
                                            0.0, buildingHeight));
              building->SetNRoomsX (1);
              building->SetNRoomsY (1);
              building->SetNFloors (1);
              buildingVector.push_back (building);
            }
        }

      // set the mobility model
      double vTx = vScatt;
      double vRx = vScatt / 2;
      txMob = CreateObject<WaypointMobilityModel> ();
      rxMob = CreateObject<WaypointMobilityModel> ();
      Time nextWaypoint = Seconds (0.0);
      txMob->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (maxAxisX / 2 - streetWidth / 2, 1.0, 1.5)));
      nextWaypoint += Seconds ((maxAxisY - streetWidth) / 2 / vTx);
      txMob->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (maxAxisX / 2 - streetWidth / 2, maxAxisY / 2 - streetWidth / 2, 1.5)));
      nextWaypoint += Seconds ((maxAxisX - streetWidth) / 2 / vTx);
      txMob->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (0.0, maxAxisY / 2 - streetWidth / 2, 1.5)));
      nextWaypoint = Seconds (0.0);
      rxMob->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (maxAxisX / 2 - streetWidth / 2, 0.0, 1.5)));
      nextWaypoint += Seconds (maxAxisY / vRx);
      rxMob->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (maxAxisX / 2 - streetWidth / 2, maxAxisY, 1.5)));

      nodes.Get (0)->AggregateObject (txMob);
      nodes.Get (1)->AggregateObject (rxMob);

      // create the channel condition model
      m_condModel = CreateObject<ThreeGppV2vUrbanChannelConditionModel> ();

      // create the propagation loss model
      m_propagationLossModel = CreateObject<ThreeGppV2vUrbanPropagationLossModel> ();
    }
  else if (scenario == "V2V-Highway")
    {
      // Two vehicles are travelling one behid the other with constant velocity
      // along the y axis. The distance between the two vehicles is 20 meters.

      // 3GPP defines that the maximum speed in urban scenario is 140 km/h
      vScatt = 140 / 3.6;
      double vTx = vScatt;
      double vRx = vScatt / 2;

      txMob = CreateObject<ConstantVelocityMobilityModel> ();
      rxMob = CreateObject<ConstantVelocityMobilityModel> ();
      txMob->GetObject<ConstantVelocityMobilityModel> ()->SetPosition (Vector (300.0, 20.0, 1.5));
      txMob->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0.0, vTx, 0.0));
      rxMob->GetObject<ConstantVelocityMobilityModel> ()->SetPosition (Vector (300.0, 0.0, 1.5));
      rxMob->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0.0, vRx, 0.0));

      nodes.Get (0)->AggregateObject (txMob);
      nodes.Get (1)->AggregateObject (rxMob);

      // create the channel condition model
      m_condModel = CreateObject<ThreeGppV2vHighwayChannelConditionModel> ();

      // create the propagation loss model
      m_propagationLossModel = CreateObject<ThreeGppV2vHighwayPropagationLossModel> ();
    }
  else
    {
      NS_FATAL_ERROR ("Unknown scenario");
    }

  m_condModel->SetAttribute ("UpdatePeriod", TimeValue (MilliSeconds (100)));

  m_propagationLossModel->SetAttribute ("Frequency", DoubleValue (frequency));
  m_propagationLossModel->SetAttribute ("ShadowingEnabled", BooleanValue (false));
  m_propagationLossModel->SetAttribute ("ChannelConditionModel", PointerValue (m_condModel));

  // create the channel model
  Ptr<ThreeGppChannelModel> channelModel = CreateObject<ThreeGppChannelModel> ();
  channelModel->SetAttribute ("Scenario", StringValue (scenario));
  channelModel->SetAttribute ("Frequency", DoubleValue (frequency));
  channelModel->SetAttribute ("ChannelConditionModel", PointerValue (m_condModel));

  // create the spectrum propagation loss model
  m_spectrumLossModel = CreateObjectWithAttributes<ThreeGppSpectrumPropagationLossModel> ("ChannelModel", PointerValue (channelModel));
  m_spectrumLossModel->SetAttribute ("vScatt", DoubleValue (vScatt));

  // initialize the devices in the ThreeGppSpectrumPropagationLossModel
  m_spectrumLossModel->AddDevice (txDev, txAntenna);
  m_spectrumLossModel->AddDevice (rxDev, rxAntenna);

  BuildingsHelper::Install (nodes);

  // set the beamforming vectors
  DoBeamforming (txDev, txAntenna, rxDev);
  DoBeamforming (rxDev, rxAntenna, txDev);

  // create the tx power spectral density
  Bands rbs;
  double freqSubBand = frequency;
  for (uint16_t n = 0; n < numRb; ++n)
    {
      BandInfo rb;
      rb.fl = freqSubBand;
      freqSubBand += subCarrierSpacing / 2;
      rb.fc = freqSubBand;
      freqSubBand += subCarrierSpacing / 2;
      rb.fh = freqSubBand;
      rbs.push_back (rb);
    }
  Ptr<SpectrumModel> spectrumModel = Create<SpectrumModel> (rbs);
  Ptr<SpectrumValue> txPsd = Create <SpectrumValue> (spectrumModel);
  double txPow_w = std::pow (10., (txPow_dbm - 30) / 10);
  double txPowDens = (txPow_w / (numRb * subCarrierSpacing));
  (*txPsd) = txPowDens;

  for (int i = 0; i < simTime / timeRes; i++)
    {
      Simulator::Schedule (timeRes * i, &ComputeSnr, txMob, rxMob, txPsd, noiseFigure);
    }

  // initialize the output file
  std::ofstream f;
  f.open ("example-output.txt", std::ios::out);
  f << "Time[s] TxPosX[m] TxPosY[m] RxPosX[m] RxPosY[m] ChannelState SNR[dB] Pathloss[dB]" << std::endl;
  f.close ();

  // print the list of buildings to file
  PrintGnuplottableBuildingListToFile ("buildings.txt");

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
