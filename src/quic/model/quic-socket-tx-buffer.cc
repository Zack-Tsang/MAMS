/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 SIGNET Lab, Department of Information Engineering, University of Padova
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
 *          Federico Chiariotti <chiariotti.federico@gmail.com>
 *          Michele Polese <michele.polese@gmail.com>
 *          Davide Marcato <davidemarcato@outlook.com>
 *
 */

#include "quic-socket-tx-buffer.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include "ns3/simulator.h"

#include "ns3/packet.h"
#include "ns3/log.h"
#include "ns3/abort.h"
#include "quic-subheader.h"
#include "quic-socket-base.h"
#include "quic-socket-tx-scheduler.h"
#include "quic-socket-tx-edf-scheduler.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("QuicSocketTxBuffer");

TypeId QuicSocketTxItem::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::QuicSocketTxItem")
    .SetParent<Object>()
    .SetGroupName ("Internet")
    .AddConstructor<QuicSocketTxItem>()
//    .AddTraceSource ("UnackSequence",
//                     "First unacknowledged sequence number (SND.UNA)",
//                     MakeTraceSourceAccessor (&QuicSocketTxBuffer::m_sentSize),
//                     "ns3::SequenceNumber32TracedValueCallback")
  ;
  return tid;
}

QuicSocketTxItem::QuicSocketTxItem () 
  : m_packet (0), 
    m_packetNumber (0), 
    m_lost (false), 
    m_retrans (false), 
    m_sacked (false), 
    m_acked (false), 
    m_isStream (false), 
    m_isStream0 (false), 
    m_lastSent (Time::Min ())
{
  m_generated = Simulator::Now ();
}

QuicSocketTxItem::QuicSocketTxItem (const QuicSocketTxItem &other)
  : m_packet (other.m_packet),
    m_packetNumber (other.m_packetNumber), 
    m_lost (other.m_lost), 
    m_retrans (other.m_retrans), 
    m_sacked (other.m_sacked), 
    m_acked (other.m_acked), 
    m_isStream (other.m_isStream), 
    m_isStream0 (other.m_isStream0), 
    m_lastSent (other.m_lastSent), 
    m_generated (other.m_generated)
{
  m_packet = other.m_packet->Copy ();
}

void QuicSocketTxItem::Print (std::ostream &os) const
{
  NS_LOG_FUNCTION (this);
  os << "[SN " << m_packetNumber.GetValue () << " - Last Sent: " << m_lastSent
     << " size " << m_packet->GetSize () << "]";

  if (m_lost)
    {
      os << "|lost|";
    }
  if (m_retrans)
    {
      os << "|retr|";
    }
  if (m_sacked)
    {
      os << "|ackd|";
    }
}

void QuicSocketTxItem::MergeItems (QuicSocketTxItem &t1, QuicSocketTxItem &t2)
{

  if (t1.m_sacked == true && t2.m_sacked == true)
    {
      t1.m_sacked = true;
    }
  else
    {
      t1.m_sacked = false;
    }
  if (t1.m_acked == true && t2.m_acked == true)
    {
      t1.m_acked = true;
    }
  else
    {
      t1.m_acked = false;
    }

  if (t2.m_retrans == true && t1.m_retrans == false)
    {
      t1.m_retrans = true;
    }
  if (t1.m_lastSent < t2.m_lastSent)
    {
      t1.m_lastSent = t2.m_lastSent;
    }
  if (t2.m_lost)
    {
      t1.m_lost = true;
    }
  if (t1.m_ackTime > t2.m_ackTime)
    {
      t1.m_ackTime = t2.m_ackTime;
    }
  if (t1.m_generated > t2.m_generated)
    {
      t1.m_generated = t2.m_generated;
    }

  t1.m_packet->AddAtEnd (t2.m_packet);
}

/**
 * zhiy zeng: Modify by ywj
 */
void QuicSocketTxItem::SplitItems (QuicSocketTxItem &t1, QuicSocketTxItem &t2,
                                   uint32_t size)
{
  uint32_t initialSize = t1.m_packet->GetSize ();

  t2.m_sacked = t1.m_sacked;
  t2.m_retrans = t1.m_retrans;
  t2.m_lastSent = t1.m_lastSent;
  t2.m_lost = t1.m_lost;
  if (t1.m_lastSent < t2.m_lastSent)
    {
      t1.m_lastSent = t2.m_lastSent;
    }
  if (t2.m_lost)
    {
      t1.m_lost = true;
    }
  t2.m_generated = t1.m_generated;
  // Copy the packet into t2
  t2.m_packet = t1.m_packet->Copy ();
  // Remove the first size bytes from t2
  t2.m_packet->RemoveAtStart (size);
  // t2.m_round = t1.m_round; // Delete by ywj, zhiy zeng

  // Change subheader
  QuicSubheader qsb;
  t1.m_packet->RemoveHeader (qsb);
  qsb.SetLength (t1.m_packet->GetSize () - size);
  t1.m_packet->AddHeader (qsb);

  NS_ASSERT_MSG (t2.m_packet->GetSize () == initialSize - size,
                 "Wrong size " << t2.m_packet->GetSize ());
  qsb.SetLength (t2.m_packet->GetSize ());
  t2.m_packet->AddHeader (qsb);
  // Remove the bytes from size to end from t1
  t1.m_packet->RemoveAtEnd (t1.m_packet->GetSize () - size);
  NS_ASSERT_MSG (t1.m_packet->GetSize () == size,
                 "Wrong size " << t1.m_packet->GetSize ());
}

NS_OBJECT_ENSURE_REGISTERED (QuicSocketTxBuffer);

TypeId QuicSocketTxBuffer::GetTypeId (void)
{
  static TypeId tid =
    TypeId ("ns3::QuicSocketTxBuffer").SetParent<Object>().SetGroupName (
      "Internet").AddConstructor<QuicSocketTxBuffer>()
//    .AddTraceSource ("UnackSequence",
//                     "First unacknowledged sequence number (SND.UNA)",
//                     MakeTraceSourceAccessor (&QuicSocketTxBuffer::m_sentSize),
//                     "ns3::SequenceNumber32TracedValueCallback")
  ;
  return tid;
}

/**
 * zhiy zeng: Modify by ywj
 */
QuicSocketTxBuffer::QuicSocketTxBuffer () :
  m_maxBuffer (32768), m_streamZeroSize (0), m_sentSize (0), m_numFrameStream0InBuffer (
    0)
{
  m_streamZeroList = QuicTxPacketList ();
  m_sentList = QuicTxPacketList (); // Add by ywj, zhiy zeng
  m_subflowSentList.insert(m_subflowSentList.end(), QuicTxPacketList ());
  // Modify by ywj, zhiy zeng
  m_subflowSentList.insert(m_subflowSentList.end(), QuicTxPacketList ());
  // m_sentList0 = QuicTxPacketList ();
  // m_sentList1 = QuicTxPacketList ();

}

/**
 * zhiy zeng: Modify by ywj
 */
QuicSocketTxBuffer::~QuicSocketTxBuffer (void)
{
  QuicTxPacketList::iterator it;

  m_sentList = QuicTxPacketList ();
  m_subflowSentList.clear();
  // m_sentList0 = QuicTxPacketList ();
  // m_sentList1 = QuicTxPacketList ();
  m_streamZeroList = QuicTxPacketList ();
  m_sentSize = 0;
  m_streamZeroSize = 0;
}

void QuicSocketTxBuffer::Print (std::ostream &os) const
{
  NS_LOG_FUNCTION (this);
  QuicSocketTxBuffer::QuicTxPacketList::const_iterator it;
  std::stringstream ss;
  std::stringstream as;

  for (it = m_subflowSentList[0].begin (); it != m_subflowSentList[0].end (); ++it)
    {
      (*it)->Print (ss);
    }

  for (it = m_streamZeroList.begin (); it != m_streamZeroList.end (); ++it)
    {
      (*it)->Print (as);
    }

  os << Simulator::Now ().GetSeconds () << "\nStream 0 list: \n" << as.str ()
     << "\n\nSent list: \n" << ss.str () << "\n\nCurrent Status: "
     << "\nNumber of transmissions = " << m_subflowSentList[0].size ()
     << "\nSent Size = " << m_sentSize
     << "\nNumber of stream 0 packets waiting = "
     << m_streamZeroList.size () << "\nStream 0 waiting packet size = "
     << m_streamZeroSize;
}

/**
 * zhiy zeng: Modify by ywj
 */
bool QuicSocketTxBuffer::Add (Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p);
  QuicSubheader qsb;

  uint32_t headerSize = p->PeekHeader (qsb);
  NS_LOG_INFO (
    "Try to append " << p->GetSize () << " bytes " << ", availSize=" << Available () << " offset " << qsb.GetOffset () << " on stream " << qsb.GetStreamId ());

  if (p->GetSize () <= Available ())
    {
      if (p->GetSize () > 0)
        {
          Ptr<QuicSocketTxItem> item = CreateObject<QuicSocketTxItem> ();
          item->m_packet = p;
          // check to which stream this packet belongs to
          uint32_t streamId = 0;
          bool isStream = false;
          if (headerSize)
            {
              streamId = qsb.GetStreamId ();
              isStream = qsb.IsStream ();
            }
          else
            {
              NS_ABORT_MSG ("No QuicSubheader in this QUIC frame " << p);
            }
          item->m_isStream = isStream;
          item->m_isStream0 = (streamId == 0);
          m_numFrameStream0InBuffer += (streamId == 0);
          if (streamId == 0)
            {
              m_streamZeroList.insert (m_streamZeroList.end (), item);
              m_streamZeroSize += item->m_packet->GetSize ();
            }
          else
            {
              m_scheduler->Add (item, false);
              // Add by ywj, zhiy zeng
              m_fileSize = qsb.GetOffset () + p->GetSize () - qsb.GetSerializedSize ();
            }

          NS_LOG_INFO (
            "Update: Application Size = " << m_scheduler->AppSize () << ", offset " << qsb.GetOffset ());
          return true;
        }
      else
        {
          NS_LOG_WARN ("Discarded. Try to insert empty packet.");
          return false;
        }
    }
  NS_LOG_WARN ("Rejected. Not enough room to buffer packet.");
  return false;
}

/**
 * zhiy zeng: Modify by ywj
 */
Ptr<Packet> QuicSocketTxBuffer::NextStream0Sequence (
  const SequenceNumber32 seq)
{
  NS_LOG_FUNCTION (this << seq);

  Ptr<QuicSocketTxItem> outItem = CreateObject<QuicSocketTxItem> ();

  QuicTxPacketList::iterator it = m_streamZeroList.begin ();
  if (it != m_streamZeroList.end ())
    {
      Ptr<Packet> currentPacket = (*it)->m_packet;
      outItem->m_packetNumber = seq;
      outItem->m_lastSent = Now ();
      outItem->m_packet = currentPacket;
      outItem->m_isStream0 = (*it)->m_isStream0;
      m_streamZeroList.erase (it);
      m_streamZeroSize -= currentPacket->GetSize ();
      //m_sentList.insert (m_sentList.end (), outItem);
      // Modify by ywj, zhiy zeng
      m_subflowSentList[0].insert (m_subflowSentList[0].end (), outItem); //ywj: only use path 0 to deal with stream 0
      m_sentSize += outItem->m_packet->GetSize ();
      ///////////////////////////////////////////////
      --m_numFrameStream0InBuffer;
      Ptr<Packet> toRet = outItem->m_packet;
      return toRet;
    }
  return 0;
}

/**
 * zhiy zeng: Modify by ywj
 */
// new
Ptr<Packet> QuicSocketTxBuffer::NextSequence (uint32_t numBytes,
                                              const SequenceNumber32 seq,
                                              uint32_t pathId,
                                              uint64_t Q,
                                              bool isFast, 
                                              bool QUpdate,
                                              uint8_t algo)
{
  NS_LOG_FUNCTION (this << numBytes << seq);

  // Modify by ywj, zhiy zeng
  Ptr<QuicSocketTxItem> outItem = GetNewSegment (numBytes, pathId, Q, isFast, QUpdate, algo);

  if (outItem != nullptr)
    {
      NS_LOG_INFO ("Extracting " << outItem->m_packet->GetSize () << " bytes");
      outItem->m_packetNumber = seq;
      outItem->m_lastSent = Now ();
      Ptr<Packet> toRet = outItem->m_packet;
      // outItem->m_round = currentRound; // Delete by ywj, zhiy zeng
      return toRet;
    }
  else
    {
      NS_LOG_INFO ("Empty packet");
      return Create<Packet>();
    }

}

// Ptr<Packet> QuicSocketTxBuffer::NextSequence (uint32_t numBytes,
//                                               const SequenceNumber32 seq)
// {
//   NS_LOG_FUNCTION (this << numBytes << seq);

//   Ptr<QuicSocketTxItem> outItem = GetNewSegment (numBytes);

//   if (outItem != nullptr)
//     {
//       NS_LOG_INFO ("Extracting " << outItem->m_packet->GetSize () << " bytes");
//       outItem->m_packetNumber = seq;
//       outItem->m_lastSent = Now ();
//       Ptr<Packet> toRet = outItem->m_packet;
//       return toRet;
//     }
//   else
//     {
//       NS_LOG_INFO ("Empty packet");
//       return Create<Packet>();
//     }

// }

/**
 * zhiy zeng: Modify by ywj
 */
Ptr<QuicSocketTxItem> QuicSocketTxBuffer::GetNewSegment (uint32_t numBytes, uint32_t pathId, uint64_t Q, bool isFast, bool QUpdate, uint8_t algo)
{
  NS_LOG_FUNCTION (this << numBytes);

  Ptr<QuicSocketTxItem> outItem = m_scheduler->GetNewSegment (numBytes,pathId,Q,isFast,QUpdate,m_fileSize,algo);

  if (outItem->m_packet->GetSize () > 0)
    {
      NS_LOG_LOGIC ("Adding packet to sent buffer");
      //m_sentList.insert (m_sentList.end (), outItem);
      m_subflowSentList[pathId].insert (m_subflowSentList[pathId].end (), outItem);
      m_sentSize += outItem->m_packet->GetSize ();
    }

  NS_LOG_INFO (
    "Update: Sent Size = " << m_sentSize << " remaining App Size " << m_scheduler->AppSize () << " object size " << outItem->m_packet->GetSize ());

  //Print(std::cout);

  return outItem;
}

/**
 * zhiy zeng: Modify by ywj
 */
//ywj: add one agurement pathId
std::vector<Ptr<QuicSocketTxItem> > QuicSocketTxBuffer::OnAckUpdate (
  Ptr<TcpSocketState> tcb, const uint32_t largestAcknowledged,
  const std::vector<uint32_t> &additionalAckBlocks,
  const std::vector<uint32_t> &gaps, uint8_t pathId)
{
  NS_LOG_FUNCTION (this);

  // findSentList (pathId);


  std::vector<uint32_t> compAckBlocks = additionalAckBlocks;
  std::vector<uint32_t> compGaps = gaps;

  std::vector<Ptr<QuicSocketTxItem> > newlyAcked;
  Ptr<QuicSocketState> tcbd = dynamic_cast<QuicSocketState*> (&(*tcb));

  compAckBlocks.insert (compAckBlocks.begin (), largestAcknowledged);
  uint32_t ackBlockCount = compAckBlocks.size ();

  std::vector<uint32_t>::const_iterator ack_it = compAckBlocks.begin ();
  std::vector<uint32_t>::const_iterator gap_it = compGaps.begin ();

  std::stringstream gap_print;
  for (auto i = gaps.begin (); i != gaps.end (); ++i)
    {
      gap_print << (*i) << " ";
    }

  std::stringstream block_print;
  for (auto i = compAckBlocks.begin (); i != compAckBlocks.end (); ++i)
    {
      block_print << (*i) << " ";
    }

  //NS_LOG_INFO ("Largest ACK: " << largestAcknowledged << ", blocks: " << block_print.str () << ", gaps: " << gap_print.str ());

  // Iterate over the ACK blocks and gaps
  for (uint32_t numAckBlockAnalyzed = 0; numAckBlockAnalyzed < ackBlockCount;
       ++numAckBlockAnalyzed, ++ack_it, ++gap_it)
    {
      for (auto sent_it = m_subflowSentList[pathId].rbegin ();
           sent_it != m_subflowSentList[pathId].rend () and !m_subflowSentList[pathId].empty (); ++sent_it)                    // Visit sentList in reverse Order for optimization
        {

          // NS_LOG_LOGIC (
          //   "Consider packet " << (*sent_it)->m_packetNumber << " (ACK block " << SequenceNumber32 ((*ack_it)) << ")");
          // The packet is in the next gap
          bool inGap = (gap_it < compGaps.end ())
            && ((*sent_it)->m_packetNumber
                <= SequenceNumber32 ((*gap_it)));
          if (inGap)               // Just for optimization we suppose All is perfectly ordered
            {
              NS_LOG_LOGIC (
                "Packet " << (*sent_it)->m_packetNumber << " missing");
              break;
            }
          // The packet is in the current block: ACK it
          //NS_LOG_LOGIC ("Packet " << (*sent_it)->m_packetNumber << " ACKed");
          bool notInGap =
            ((gap_it >= compGaps.end ())
             || ((*sent_it)->m_packetNumber
                 > SequenceNumber32 ((*gap_it))));

          if ((*sent_it)->m_packetNumber <= SequenceNumber32 ((*ack_it))
              and notInGap and (*sent_it)->m_sacked == false)
            {
              (*sent_it)->m_sacked = true;
              (*sent_it)->m_ackTime = Now ();
              newlyAcked.push_back ((*sent_it));
              UpdateRateSample ((*sent_it));
            }

        }
    }
  NS_LOG_LOGIC ("Mark lost packets");
  // Mark packets as lost as in RFC (Sec. 4.2.1 of draft-ietf-quic-recovery-15)
  uint32_t index = m_subflowSentList[pathId].size ();
  bool lost = false;
  bool outstanding = false;
  auto acked_it = m_subflowSentList[pathId].rend ();
  // Iterate over the sent packet list in reverse
  for (auto sent_it = m_subflowSentList[pathId].rbegin ();
       sent_it != m_subflowSentList[pathId].rend () and !m_subflowSentList[pathId].empty ();
       ++sent_it, --index)
    {
      // All previous packets are lost
      if (lost)
        {
          if (!(*sent_it)->m_sacked)
            {
              (*sent_it)->m_lost = true;
              NS_LOG_LOGIC (
                "Packet " << (*sent_it)->m_packetNumber << " lost");
            }
        }
      else
        {
          // The packet is the last ACKed packet
          if ((*sent_it)->m_packetNumber.GetValue () == largestAcknowledged)
            {
              // Mark the packet as ACKed
              acked_it = sent_it;
              outstanding = true;
            }
          else if (outstanding && !(*sent_it)->m_sacked)
            {
              //ACK-based detection
              if (largestAcknowledged - (*sent_it)->m_packetNumber.GetValue ()
                  >= tcbd->m_kReorderingThreshold)
                {
                  (*sent_it)->m_lost = true;
                  lost = true;
                  NS_LOG_INFO (
                    "Largest ACK " << largestAcknowledged << ", lost packet " << (*sent_it)->m_packetNumber.GetValue () << " - reordering " << tcbd->m_kReorderingThreshold);
                }
              // Time-based detection (optional)
              if (tcbd->m_kUsingTimeLossDetection)
                {
                  double lhsComparison = ((*acked_it)->m_ackTime
                                          - (*sent_it)->m_lastSent).GetSeconds ();
                  double rhsComparison = tcbd->m_kTimeReorderingFraction
                    * tcbd->m_smoothedRtt.GetSeconds ();
                  if (lhsComparison >= rhsComparison)
                    {
                      NS_LOG_UNCOND (
                        "Largest ACK " << largestAcknowledged << ", lost packet " << (*sent_it)->m_packetNumber.GetValue () << " - time " << rhsComparison);
                      (*sent_it)->m_lost = true;
                      lost = true;
                    }
                }
            }
        }
    }

  // Clean up acked packets and return new ACKed packet vector
  CleanSentList (pathId);
  return newlyAcked;
}

//ywj: ResetSentList (uint32_t keepItems = 1) => ResetSentList (uint8_t pathId, uint32_t keepItems = 1)
void QuicSocketTxBuffer::ResetSentList (uint8_t pathId, uint32_t keepItems)
{
  NS_LOG_FUNCTION (this << keepItems);
  uint32_t kept = 0;

  // findSentList (pathId);

  for (auto sent_it = m_subflowSentList[pathId].rbegin ();
       sent_it != m_subflowSentList[pathId].rend () and !m_subflowSentList[pathId].empty ();
       ++sent_it, kept++)
    {
      if (kept >= keepItems && !(*sent_it)->m_sacked)
        {
          (*sent_it)->m_lost = true;
        }
    }
}

/**
 * zhiy zeng: Modify by ywj
 */
bool QuicSocketTxBuffer::MarkAsLost (const SequenceNumber32 seq)
{
  NS_LOG_FUNCTION (this << seq);
  bool found = false;
  //ywj: just comment out the following lines of code, as nowhere use it other than quic/test/quic-tx-buffer-test.cc
  // for (auto sent_it = m_sentList.begin ();
  //      sent_it != m_sentList.end () and !m_sentList.empty (); ++sent_it)
  //   {
  //     if ((*sent_it)->m_packetNumber == seq)
  //       {
  //         found = true;
  //         (*sent_it)->m_lost = true;
  //       }
  //   }
  return found;
}

/**
 * zhiy zeng: Modify by ywj
 */
uint32_t QuicSocketTxBuffer::Retransmission (SequenceNumber32 packetNumber, uint8_t pathId)
{
  NS_LOG_FUNCTION (this);
  uint32_t toRetx = 0;

  // findSentList (pathId);
  // First pass: add lost packets to the application buffer
  for (auto sent_it = m_subflowSentList[pathId].rbegin (); sent_it != m_subflowSentList[pathId].rend ();
       ++sent_it)
    {
      Ptr<QuicSocketTxItem> item = *sent_it;
      if (item->m_lost)
        {
          // Add lost packet contents to app buffer
          Ptr<QuicSocketTxItem> retx = CreateObject<QuicSocketTxItem> ();
          retx->m_packetNumber = packetNumber++;
          retx->m_isStream = item->m_isStream;
          retx->m_isStream0 = item->m_isStream0;
          retx->m_packet = Create<Packet>();
          NS_LOG_INFO (
            "Retx packet " << item->m_packetNumber << " as " << retx->m_packetNumber.GetValue ());
          // Modify by ywj, zhiy zeng
          std::cout<<Simulator::Now().GetSeconds()
                    <<" QuicSocketTxBuffer::Retransmission "
                    <<"Retx packet " << item->m_packetNumber << " as " << retx->m_packetNumber.GetValue ()<<std::endl;
          QuicSocketTxItem::MergeItems (*retx, *item);
          retx->m_lost = false;
          retx->m_retrans = true;
          toRetx += retx->m_packet->GetSize ();
          // Modify by ywj, zhiy zeng
          m_sentSize -= retx->m_packet->GetSize ();
          if (retx->m_isStream0)
            {
              NS_LOG_INFO ("Lost stream 0 packet, re-inserting in list");
              m_streamZeroList.insert (m_streamZeroList.begin (), retx);
              m_streamZeroSize += retx->m_packet->GetSize ();
              m_numFrameStream0InBuffer++;
            }
          else
            {
              m_scheduler->Add (retx, true);
            }
        }
    }

  NS_LOG_LOGIC ("Remove retransmitted packets from sent list");
  auto sent_it = m_subflowSentList[pathId].begin ();
  // Remove lost packets from the sent list
  while (!m_subflowSentList[pathId].empty () && sent_it != m_subflowSentList[pathId].end ())
    {
      Ptr<QuicSocketTxItem> item = *sent_it;
      if (item->m_lost)
        {
          // Remove lost packet from sent vector
          sent_it = m_subflowSentList[pathId].erase (sent_it);
        }
      else
        {
          sent_it++;
        }
    }
  return toRetx;
}

std::vector<Ptr<QuicSocketTxItem> > QuicSocketTxBuffer::DetectLostPackets (uint8_t pathId)
{
  NS_LOG_FUNCTION (this);
  std::vector<Ptr<QuicSocketTxItem> > lost;

  // findSentList (pathId);

  for (auto sent_it = m_subflowSentList[pathId].begin ();
       sent_it != m_subflowSentList[pathId].end () and !m_subflowSentList[pathId].empty (); ++sent_it)
    {
      if ((*sent_it)->m_lost)
        {
          lost.push_back ((*sent_it));
          NS_LOG_INFO ("Packet " << (*sent_it)->m_packetNumber << " is lost");
          std::cout<<Simulator::Now().GetSeconds()
                    <<" QuicSocketTxBuffer::DetectLostPackets "
                    <<" Packet " << (*sent_it)->m_packetNumber << " is lost"<<std::endl;
        }
    }
  return lost;
}

uint32_t QuicSocketTxBuffer::GetLost (uint8_t pathId)
{
  NS_LOG_FUNCTION (this);
  uint32_t lostCount = 0;

  // findSentList (pathId);

  for (auto sent_it = m_subflowSentList[pathId].begin ();
       sent_it != m_subflowSentList[pathId].end () and !m_subflowSentList[pathId].empty (); ++sent_it)
    {
      if ((*sent_it)->m_lost)
        {
          lostCount += (*sent_it)->m_packet->GetSize ();
        }
    }
  return lostCount;
}

/**
 * zhiy zeng: Modify by ywj
 */
void QuicSocketTxBuffer::CleanSentList (uint8_t pathId)
{
  NS_LOG_FUNCTION (this);
  // findSentList (pathId);
  auto sent_it = m_subflowSentList[pathId].begin ();
  // All packets up to here are ACKed (already sent to the receiver app)
  while (!m_subflowSentList[pathId].empty () && (*sent_it)->m_sacked && !(*sent_it)->m_lost)
    {
      // Remove ACKed packet from sent vector
      Ptr<QuicSocketTxItem> item = *sent_it;
      item->m_acked = true;
      m_sentSize -= item->m_packet->GetSize (); /// Modify by ywj, zhiy zeng
      m_subflowSentList[pathId].erase (sent_it);
      // Delete by ywj, zhiy zeng
      //NS_LOG_LOGIC ("Packet " << (*sent_it)->m_packetNumber << " received and ACKed. Removing from sent buffer");
      sent_it = m_subflowSentList[pathId].begin ();
    }
}

uint32_t QuicSocketTxBuffer::Available (void) const
{
  return m_maxBuffer - m_streamZeroSize - m_scheduler->AppSize ();
}

uint32_t QuicSocketTxBuffer::GetMaxBufferSize (void) const
{
  return m_maxBuffer;
}

void QuicSocketTxBuffer::SetMaxBufferSize (uint32_t n)
{
  m_maxBuffer = n;
}

uint32_t QuicSocketTxBuffer::AppSize (void) const
{
  return m_streamZeroSize + m_scheduler->AppSize ();
}

/**
 * zhiy zeng: Add by ywj
 */
uint32_t QuicSocketTxBuffer::FileSize (void) const
{
  return m_scheduler->FileSize ();
}

/**
 * zhiy zeng: Add by ywj
 */
uint32_t QuicSocketTxBuffer::SizeOnSlowPath (void) const
{
  return m_scheduler->SizeOnSlowPath ();
}

uint32_t QuicSocketTxBuffer::GetNumFrameStream0InBuffer (void) const
{
  return m_numFrameStream0InBuffer;
}

//ywj: new defined
// void QuicSocketTxBuffer::findSentList (uint8_t pathId)
// {
//   NS_LOG_FUNCTION (this);

//   if (pathId == 0){
//     refList (m_sentList0);
//     //m_sentList = m_sentList0;
//   }else if (pathId == 1){
//     refList (m_sentList1);
//     //m_sentList = m_sentList1;
//   }else{
//     NS_LOG_ERROR("wrong pathId!!!");
//   }
// }

//ywj: pass m_sentList0 or m_sentList1 by reference to m_sentList
// void QuicSocketTxBuffer::refList (QuicTxPacketList & sentList){

//   NS_LOG_FUNCTION (this);
//   m_sentList = sentList;
// }

//ywj: BytesInFlight () => BytesInFlight (uint8_t pathId) 
uint32_t QuicSocketTxBuffer::BytesInFlight (uint8_t pathId) 
{
  NS_LOG_FUNCTION (this);

  uint32_t inFlight = 0;

  // findSentList (pathId);

  for (auto sent_it = m_subflowSentList[pathId].begin ();
       sent_it != m_subflowSentList[pathId].end () and !m_subflowSentList[pathId].empty (); ++sent_it)
    {
      if (!(*sent_it)->m_isStream0 && (*sent_it)->m_isStream
          && !(*sent_it)->m_sacked)
        {
          inFlight += (*sent_it)->m_packet->GetSize ();
        }
    }

  NS_LOG_INFO (
    "Compute bytes in flight " << inFlight << " m_sentSize " << m_sentSize << " m_appSize " << m_streamZeroSize + m_scheduler->AppSize ());
  return inFlight;

}

/**
 * zhiy zeng: Add by ywj
 */
void QuicSocketTxBuffer::SetQuicSocketState (Ptr<QuicSocketState> tcb)
{
  NS_LOG_FUNCTION (this);
  m_tcb = tcb;
}

void QuicSocketTxBuffer::SetScheduler (Ptr<QuicSocketTxScheduler> sched)
{
  NS_LOG_FUNCTION (this);
  m_scheduler = sched;
}

/**
 * zhiy zeng: Modify by ywj
 */
void QuicSocketTxBuffer::UpdatePacketSent (SequenceNumber32 seq, uint32_t sz, uint8_t pathId)
{
  NS_LOG_FUNCTION (this << seq << sz);

  // findSentList (pathId);

  if (m_tcb == nullptr or sz == 0)
    {
      return;
    }

  if (m_tcb->m_bytesInFlight.Get () == 0)
    {
      m_tcb->m_firstSentTime = Simulator::Now ();
      m_tcb->m_deliveredTime = Simulator::Now ();
    }

  Ptr<QuicSocketTxItem> item = nullptr;
  for (auto it = m_subflowSentList[pathId].rbegin (); it != m_subflowSentList[pathId].rend (); ++it)
    {
      if ((*it)->m_packetNumber == seq)
        {
          item = *it;
          break;
        }
    }
  NS_ASSERT_MSG (item != nullptr, "not found seq " << seq);
  item->m_firstSentTime = m_tcb->m_firstSentTime;
  item->m_deliveredTime = m_tcb->m_deliveredTime;
  item->m_isAppLimited = (m_tcb->m_appLimitedUntil > m_tcb->m_delivered);
  item->m_delivered = m_tcb->m_delivered;
  item->m_ackBytesSent = m_tcb->m_ackBytesSent;
}

void
QuicSocketTxBuffer::UpdateAckSent (SequenceNumber32 seq, uint32_t sz)
{
  if (m_tcb == nullptr or sz == 0)
    {
      return;
    }

  m_tcb->m_ackBytesSent += sz;
}

struct RateSample*
QuicSocketTxBuffer::GetRateSample ()
{
  NS_LOG_FUNCTION (this);
  return &m_rs;
}

void
QuicSocketTxBuffer::UpdateRateSample (Ptr<QuicSocketTxItem> item)
{
  NS_LOG_FUNCTION (this << item);

  if (m_tcb == nullptr or item->m_deliveredTime == Time::Max ())
    {
      // item already SACKed
      return;
    }

  m_tcb->m_delivered         += item->m_packet->GetSize ();
  m_tcb->m_deliveredTime      = Simulator::Now ();

  if (item->m_delivered > m_rs.m_priorDelivered)
    {
      m_rs.m_priorDelivered   = item->m_delivered;
      m_rs.m_priorTime        = item->m_deliveredTime;
      m_rs.m_isAppLimited     = item->m_isAppLimited;
      m_rs.m_sendElapsed      = item->m_lastSent - item->m_firstSentTime;
      m_rs.m_ackElapsed       = m_tcb->m_deliveredTime - item->m_deliveredTime;
      m_tcb->m_firstSentTime  = item->m_lastSent;
      m_rs.m_priorAckBytesSent  = item->m_ackBytesSent;
    }

  /* Mark the packet as delivered once it is SACKed to avoid
   * being used again when it's cumulatively acked.
   */
  item->m_deliveredTime = Time::Max ();
  m_tcb->m_txItemDelivered = item->m_delivered;
}

bool
QuicSocketTxBuffer::GenerateRateSample ()
{
  NS_LOG_FUNCTION (this);

  if (m_tcb == nullptr)
    {
      return false;
    }

  if (m_rs.m_priorTime == Seconds (0))
    {
      return false;
    }

  m_rs.m_interval = std::max (m_rs.m_sendElapsed, m_rs.m_ackElapsed);

  m_rs.m_delivered = m_tcb->m_delivered - m_rs.m_priorDelivered;


  if (m_rs.m_ackBytesSent < m_tcb->m_ackBytesSent - m_rs.m_priorAckBytesSent or++ m_rs.m_ackBytesMaxWin > 5) //quick maxfilter implementation
    {
      m_rs.m_ackBytesSent = m_tcb->m_ackBytesSent - m_rs.m_priorAckBytesSent;
      m_rs.m_ackBytesMaxWin = 0;
    }

  uint32_t discountedDelivered = m_rs.m_delivered > m_rs.m_ackBytesSent ? m_rs.m_delivered - m_rs.m_ackBytesSent : 0U;

  if (m_rs.m_interval < m_tcb->m_minRtt)
    {
      m_rs.m_interval = Seconds (0);
      return false;
    }

  if (m_rs.m_interval != Seconds (0))
    {
      m_rs.m_deliveryRate = DataRate (discountedDelivered * 8.0 / m_rs.m_interval.GetSeconds ());
    }
  NS_LOG_DEBUG ("computed delivery rate: " << m_rs.m_deliveryRate);
  return true;
}


void QuicSocketTxBuffer::SetLatency (uint32_t streamId, Time latency)
{
  // Only relevant for the EDF scheduler
  if (m_scheduler->GetTypeId () == QuicSocketTxEdfScheduler::GetTypeId ())
    {
      (DynamicCast<QuicSocketTxEdfScheduler> (m_scheduler))->SetLatency (streamId, latency);
    }
}

Time QuicSocketTxBuffer::GetLatency (uint32_t streamId)
{
  // Only relevant for the EDF scheduler
  if (m_scheduler->GetTypeId () == QuicSocketTxEdfScheduler::GetTypeId ())
    {
      return (DynamicCast<QuicSocketTxEdfScheduler> (m_scheduler))->GetLatency (streamId);
    }
  else
    {
      return Seconds (0);
    }
}

void QuicSocketTxBuffer::SetDefaultLatency (Time latency)
{
  // Only relevant for the EDF scheduler
  if (m_scheduler->GetTypeId () == QuicSocketTxEdfScheduler::GetTypeId ())
    {
      (DynamicCast<QuicSocketTxEdfScheduler> (m_scheduler))->SetDefaultLatency (latency);
    }
}

Time QuicSocketTxBuffer::GetDefaultLatency ()
{
  return GetLatency (0);
}

}
