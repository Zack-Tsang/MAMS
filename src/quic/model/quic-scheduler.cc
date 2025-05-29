


#include "ns3/object.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/nstime.h"
#include "quic-stream.h"
#include "ns3/node.h"
#include "quic-scheduler.h"
#include "ns3/string.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <vector>
#include <boost/random.hpp>
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

/**
 * zhiy zeng: 在 MP-QUIC（Multipath QUIC）协议栈中进行路径调度决策
 * 实现了多种调度策略：
  * 轮询（Round Robin）
  * 加权轮询（Weighted Round Robin）
  * 基于RTT的调度（RTT-based）
  * 基于多臂老虎机算法（Multi-Armed Bandit Algorithm）
 * 
 */
namespace ns3 { // 所属命名空间

NS_LOG_COMPONENT_DEFINE ("QuicScheduler"); // 日志组件定义

NS_OBJECT_ENSURE_REGISTERED (QuicScheduler); // 注册QuicScheduler类


TypeId
QuicScheduler::GetTypeId (void)
{
  // 定义一个静态的TypeId对象, 对应的是类QuicScheduler, 位于命名空间ns3中
  static TypeId tid = TypeId ("ns3::QuicScheduler") 
    .SetParent<Object> () // 继承自Object类
    .SetGroupName ("Internet") // 设置归属组
    .AddAttribute ("SchedulerType", // 添加名为SchedulerType的属性
                   "Minimum time in the future an RTO alarm may be set for",
                   StringValue ("RR"), // 默认值为字符串"RR"
                   // 关联到成员变量m_type, 可通过m_type访问
                   MakeStringAccessor (&QuicScheduler::m_type),
                   MakeStringChecker ())
                   
  ;
  return tid;
}

/**
 * zhiy zeng: 构造函数, 用于初始化成员变量
 */
QuicScheduler::QuicScheduler ()
  : Object () // 调用父类构造函数
{
  NS_LOG_FUNCTION_NOARGS ();
  m_sendIndex = 0; // 当前发送索引, 用于轮询调度
  m_pid = 0; // 最小延迟路径 ID（由 SetMinPath() 设置）

  // Probability of winning for each bandit. Change this variable to experiment
  // with different probabilities of winning.
  p = {0.5, 0.5}; // 每个路径的“赢率”（MAB 算法使用）

  // Number of trials per bandit
  trials = std::vector<unsigned int>(p.size()); // 每条路径的尝试次数
  // Number of wins per bandif
  wins = std::vector<unsigned int>(p.size()); // 每条路径的成功次数 (选中次数)
  // Beta distributions of the priors for each bandit
  
  // Initialize the prior distributions with alpha=1 beta=1
  // MAB算法的先验分布初始化为 Beta(1, 1)
  for (size_t i = 0; i < p.size(); i++) {
    prior_dists.push_back(boost::random::beta_distribution<>(1, 1));
  }
}

/**
 * zhiy zeng: 析构函数, 用于释放资源
 */
QuicScheduler::~QuicScheduler ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_sendIndex = 0;
  m_pid = 0;
}

// void
// QuicScheduler::SetSubflows(std::vector <MpQuicSubFlow *> subflows)
// {
//   // m_subflows = subflows;
//   return;
// }

/**
 * zhiy zeng: 获取发送路径索引
 * 通过不同的调度算法（轮询、加权轮询、RTT、MAB）来选择发送路径
 * @return 发送路径索引
 */
int
QuicScheduler::GetSend()
{
  if(m_sendIndex == 7000) // 7000次后重置
  
  {
    SetSendDefault(); // 使用路径0
  }
  if (m_type == "weight")
  {
    return WeightedRoundRobin(); // 加权RR
  } 
  else if (m_type == "rtt")
  {
    return RttBase(); // minRTT
  }
  else if (m_type == "bandit")
  {
    return MabAlog(); // MAB
  }
  else
  {
    return RoundRobin(); // RR
  }
}

/**
 * zhiy zeng: 设置发送路径索引为0
 * 即使用路径0进行发送
 */
void
QuicScheduler::SetSendDefault()
{
  m_sendIndex = 0;
}

/**
 * zhiy zeng: 将调度算法类型设置为加权轮询
 */
void
QuicScheduler::SetWeightType()
{
  m_type = "weight";
}

/**
 * zhiy zeng: 轮询调度算法
 * 轮询调度算法会在每次调用时返回 0 和 1 交替
 * 即只支持2条路径，且未进行路径拥塞窗口可用性判断
 * @return 发送路径索引
 */
int
QuicScheduler::RoundRobin()
{
  m_sendIndex++;

  if(m_sendIndex%2 == 0 )
  {
    return 0;
  } 
  else 
  {
    return 1;
  }
}

/**
 * zhiy zeng: 加权轮询调度算法
 * 路径 0 被优先使用一段时间（index 2~10）
 * 之后路径 1 被使用
 * 仅支持2条路径
 * @note: 该算法未进行路径拥塞窗口可用性判断
 * @return 发送路径索引
 */
int
QuicScheduler::WeightedRoundRobin()
{
  // std::cout<<m_sendIndex<<" index \n";
  m_sendIndex++;
  if(m_sendIndex >=2 && m_sendIndex<= 10)
  {
    return 0;
  } 
  else 
  {
    return 1;
  }
}

/**
 * zhiy zeng: RTT调度算法
 * 在初始阶段（m_sendIndex <= 1）时，轮询选择路径
 * 在非初始阶段时，使用最小RTT路径
 * @note 该算法未进行路径拥塞窗口可用性判断
 * @return 发送路径索引
 */
int
QuicScheduler::RttBase()
{
  m_sendIndex++;
  // 非初始阶段
  if(m_sendIndex > 1)
  {
    return m_pid;
  }
  // 初始阶段: 轮询选择路径
  if(m_sendIndex%2 == 0)
  {
    return 1;
  } 
  else 
  {
    return 0;
  }
}

/**
 * zhiy zeng: 设置最小RTT路径
 * @param p 最小RTT路径索引
 */
void
QuicScheduler::SetMinPath(int p)
{
  m_pid = p;
}
// int
// QuicScheduler::FindMinRttPath()
// {
//   int min = 0;
//   Time mrtt=m_subflows[0]->lastMeasuredRtt;
//   for (uint i = 1; i < m_subflows.size(); i++)
//   {
//     if (m_subflows[i]->lastMeasuredRtt < mrtt){
//       mrtt = m_subflows[i]->lastMeasuredRtt;
//       min = i;
//     }
//   }
//   return min;
// }


// pull_lever has a chance of 1/weight of returning 1.
/**
 * zhiy zeng: 模拟路径的成功概率（根据传入的 weight）返回是否成功（1/0）
 * @param gen 随机数生成器
 * @param weight 成功概率
 * @return 1表示成功，0表示失败
 */
unsigned int 
QuicScheduler::pull_lever(base_generator *gen, double weight) {
  double probabilities[] = {1-weight, weight};
  boost::random::discrete_distribution<> dist(probabilities);
  return dist(*gen);
}

// argmax returns the index of maximum element in vector v.
/**
 * zhiy zeng: 返回向量中最大元素的索引
 * @param v 输入向量
 * @return 最大元素的索引
 */
size_t 
QuicScheduler::argmax(const std::vector<double>& v){
  return std::distance(v.begin(), std::max_element(v.begin(), v.end()));
}

/**
 * zhiy zeng: 多臂老虎机算法（MAB）调度算法
 * 使用贝叶斯UCB方法来选择路径, 基于 Beta 分布建模每条路径的性能
 * 每次从当前最优估计路径中选择一个发送
 * 根据结果更新其 Beta 分布（Bayesian Update）
 * 实现了 探索-利用（Exploration-Exploitation） 平衡
 */
int
QuicScheduler::MabAlog(){
  
  // gen is a Mersenne Twister random generator. We initialzie it here to keep
  // the binary deterministic.
  
  std::vector<double> priors; // 先验分布
  // Sample a random value from each prior distribution.
  for (auto& dist : prior_dists) {
    priors.push_back(dist(gen)); // 使用随机数生成器生成先验分布
  }
  // Select the bandit that has the highest sampled value from the prior
  size_t chosen_bandit = argmax(priors); // 选择先验分布中最大值的索引
  trials[chosen_bandit]++; // 更新尝试次数
  // Pull the lever of the chosen bandit
  wins[chosen_bandit] += pull_lever(&gen, p[chosen_bandit]); // 更新成功次数

  // Update the prior distribution of the chosen bandit
  auto alpha = 1 + wins[chosen_bandit]; // 更新分布
  auto beta = 1 + trials[chosen_bandit] - wins[chosen_bandit]; // 更新分布
  // 更新先验分布
  prior_dists[chosen_bandit] = boost::random::beta_distribution<>(alpha, beta);
  
  // std::cout << "chosen bandit: " << chosen_bandit << std::endl;
  // 返回选择的路径索引
  return chosen_bandit;
  // auto sp = std::cout.precision();
  // std::cout << std::setprecision(3);
  // for (size_t i = 0; i < p.size(); i++) {
  //   std::cout << "Bandit " << i+1 << ": ";
  //   double empirical_p = double(wins[i]) / trials[i];
  //   std::cout << "wins/trials: " << wins[i] << "/" << trials[i] << ". ";
  //   std::cout << "Estimated p: " << empirical_p << " ";
  //   std::cout << "Actual p: " << p[i] << std::endl;
  // }
  // std::cout << std::endl;
  // auto expected_optimal_wins = *std::max_element(p.begin(), p.end()) * runs;
  // std::cout << std::setprecision(sp);
  // std::cout << "Expected number of wins with optimal strategy: " << expected_optimal_wins << std::endl;
  // std::cout << "Actual wins: " << std::accumulate(wins.begin(), wins.end(), 0) << std::endl;

}


} // namespace ns3
