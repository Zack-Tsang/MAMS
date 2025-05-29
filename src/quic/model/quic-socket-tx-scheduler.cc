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
 * Authors: Federico Chiariotti <chiariotti.federico@gmail.com>
 *          Michele Polese <michele.polese@gmail.com>
 *          Umberto Paro <umberto.paro@me.com>
 *
 */

#include "quic-socket-tx-scheduler.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include "ns3/simulator.h"

#include "ns3/packet.h"
#include "ns3/log.h"
#include "ns3/abort.h"
#include "quic-subheader.h"
#include "quic-socket-tx-buffer.h"
#include "quic-socket-base.h"

// Add by ywj, zhiy zeng
#include <iomanip>
#include <cstring>
#include <stdlib.h>
#include <unistd.h>
#include <regex>
/////////////////////////
namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("QuicSocketTxScheduler");

NS_OBJECT_ENSURE_REGISTERED (QuicSocketTxScheduler);
NS_OBJECT_ENSURE_REGISTERED (QuicSocketTxScheduleItem);

TypeId
QuicSocketTxScheduleItem::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::QuicSocketTxScheduleItem")
    .SetParent<Object> ()
    .SetGroupName ("Internet")
  ;
  return tid;
}

int
QuicSocketTxScheduleItem::Compare (const QuicSocketTxScheduleItem & o) const
{
  if (m_priority != o.m_priority)
    {
      return (m_priority < o.m_priority) ? -1 : 1;
    }
  if (m_streamId != o.m_streamId)
    {
      return (m_streamId < o.m_streamId) ? -1 : 1;
    }
  if (m_offset != o.m_offset)
    {
      return (m_offset < o.m_offset) ? -1 : 1;
    }

  return 0;
}



QuicSocketTxScheduleItem::QuicSocketTxScheduleItem (uint64_t id, uint64_t off, double p, Ptr<QuicSocketTxItem> it)
  : m_streamId (id), 
    m_offset (off), 
    m_priority (p), 
    m_item (it)
{}

QuicSocketTxScheduleItem::QuicSocketTxScheduleItem (const QuicSocketTxScheduleItem &other)
  : m_streamId (other.m_streamId), 
    m_offset (other.m_offset), 
    m_priority (other.m_priority)
{
  m_item = CreateObject<QuicSocketTxItem> (*(other.m_item));
}


Ptr<QuicSocketTxItem>
QuicSocketTxScheduleItem::GetItem () const
{
  return m_item;
}

uint64_t
QuicSocketTxScheduleItem::GetStreamId () const
{
  return m_streamId;
}


uint64_t
QuicSocketTxScheduleItem::GetOffset () const
{
  return m_offset;
}

double
QuicSocketTxScheduleItem::GetPriority () const
{
  return m_priority;
}

void
QuicSocketTxScheduleItem::SetPriority (double priority)
{
  m_priority = priority;
}



TypeId
QuicSocketTxScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::QuicSocketTxScheduler")
    .SetParent<Object> ()
    .SetGroupName ("Internet")
    .AddConstructor<QuicSocketTxScheduler> ()
  ;
  return tid;
}

QuicSocketTxScheduler::QuicSocketTxScheduler () : m_appSize (0)
{
  m_appList = QuicTxPacketList ();
}

QuicSocketTxScheduler::QuicSocketTxScheduler (const QuicSocketTxScheduler &other) : m_appSize (other.m_appSize)
{
  m_appList = other.m_appList;
}

QuicSocketTxScheduler::~QuicSocketTxScheduler (void)
{
  m_appList = QuicTxPacketList ();
  m_appSize = 0;
}


void
QuicSocketTxScheduler::Add (Ptr<QuicSocketTxItem> item, bool retx)
{
  NS_LOG_FUNCTION (this << item);
  QuicSubheader qsb;
  item->m_packet->PeekHeader (qsb);
  double priority = -1;
  NS_LOG_INFO ("Adding packet on stream " << qsb.GetStreamId ());
  if (!retx)
    {
      NS_LOG_INFO ("Standard item, add at end (offset " << qsb.GetOffset () << ")");
      priority = Simulator::Now ().GetSeconds ();
    }
  else
    {
      NS_LOG_INFO ("Retransmitted item, add at beginning (offset " << qsb.GetOffset () << ")");
      std::cout<<"time: "<<Simulator::Now().GetSeconds()
                <<" QuicSocketTxScheduler::Add "
                <<"Retransmitted item, add at beginning (offset " << qsb.GetOffset () << ")"<<std::endl;
    }
  Ptr<QuicSocketTxScheduleItem> sched = CreateObject<QuicSocketTxScheduleItem> (qsb.GetStreamId (), qsb.GetOffset (), priority, item);
  AddScheduleItem (sched, retx);
}


void
QuicSocketTxScheduler::AddScheduleItem (Ptr<QuicSocketTxScheduleItem> item, bool retx)
{
  NS_LOG_FUNCTION (this << item);
  m_appList.push (item);
  m_appSize += item->GetItem ()->m_packet->GetSize ();
  QuicSubheader qsb;
  item->GetItem ()->m_packet->PeekHeader (qsb);
  NS_LOG_INFO ("Adding packet on stream " << qsb.GetStreamId () << " with priority " << item->GetPriority ());
  if (!retx)
    {
      NS_LOG_INFO ("Standard item, add at end (offset " << qsb.GetOffset () << ")");
    }
  else
    {
      NS_LOG_INFO ("Retransmitted item, add at beginning (offset " << qsb.GetOffset () << ")");
    }
}
/* Ptr<QuicSocketTxItem>
QuicSocketTxScheduler::GetNewSegment (uint32_t numBytes, uint32_t pathId, uint64_t Q, bool isFast, bool QUpdate, uint32_t fileSize, uint8_t algo)
{
  NS_LOG_FUNCTION (this << numBytes);


  bool firstSegment = true;
  Ptr<Packet> currentPacket = 0;
  Ptr<QuicSocketTxItem> currentItem = 0;
  Ptr<QuicSocketTxItem> outItem = CreateObject<QuicSocketTxItem>();
  outItem->m_isStream = true;   // Packets sent with this method are always stream packets
  outItem->m_isStream0 = false;
  outItem->m_packet = Create<Packet> ();
  uint32_t outItemSize = 0;

  //     Below we try making sure that we don't split/combine retx packets.
  //      This simplifies the logic on the Rx side which has been modified to account for the 
  //      possibility that duplicate stream data will be received due to lost ACKs/spurious
  //      retransmissions. 
  if (m_appSize > 0) 
    {
      Ptr<QuicSocketTxScheduleItem> firstScheduleItem = m_appList.top();
      Ptr<QuicSocketTxItem> txItem = firstScheduleItem->GetItem();
      Ptr<Packet> packet = txItem->m_packet;
      auto packetSize = packet->GetSize();

      bool isRetx = firstScheduleItem->GetPriority() == -1;
      if (isRetx) 
        {
          NS_LOG_DEBUG ("Next packet is ReTx packet " << txItem->m_packetNumber);

          NS_LOG_DEBUG ("Returning single ReTx item for packet " << txItem->m_packetNumber);

          m_appSize -= packetSize;
          m_appList.pop();

          QuicSocketTxItem::MergeItems (*outItem, *txItem);
          outItemSize += packetSize;

          NS_LOG_INFO ("Update: remaining App Size " << m_appSize << ", object size " << outItemSize);
          return outItem;
        }
    }

  while (m_appSize > 0 && outItemSize < numBytes)
    {
      Ptr<QuicSocketTxScheduleItem> scheduleItem = m_appList.top ();
      currentItem = scheduleItem->GetItem ();
      currentPacket = currentItem->m_packet;
      m_appSize -= currentPacket->GetSize ();
      m_appList.pop ();

      if (outItemSize + currentItem->m_packet->GetSize ()   
          <= numBytes)       // Merge
        {
          NS_LOG_LOGIC ("Add complete frame to the outItem - size "
                        << currentItem->m_packet->GetSize ()
                        << " m_appSize " << m_appSize);
          QuicSubheader qsb;
          currentPacket->PeekHeader (qsb);
          NS_LOG_INFO ("Packet: stream " << qsb.GetStreamId () << ", offset " << qsb.GetOffset ());
          QuicSocketTxItem::MergeItems (*outItem, *currentItem);
          outItemSize += currentItem->m_packet->GetSize ();

          NS_LOG_LOGIC ("Updating application buffer size: " << m_appSize);
          continue;
        }
      else if (firstSegment)  // we cannot transmit a full packet, so let's split it and update the subheaders
        {
          firstSegment = false;

          // get the currentPacket subheader
          QuicSubheader qsb;
          currentPacket->PeekHeader (qsb);

          // new packet size
          int newPacketSizeInt = (int)numBytes - outItemSize - qsb.GetSerializedSize ();
          if (newPacketSizeInt <= 0)
            {
              NS_LOG_INFO ("Not enough bytes even for the header");
              m_appList.push (scheduleItem);
              m_appSize += currentPacket->GetSize ();
              break;
            }
          else
            {
              NS_LOG_INFO ("Split packet on stream " << qsb.GetStreamId () << ", sending " << newPacketSizeInt << " bytes from offset " << qsb.GetOffset ());

              currentPacket->RemoveHeader (qsb);
              uint32_t newPacketSize = (uint32_t)newPacketSizeInt;

              NS_LOG_LOGIC ("Add incomplete frame to the outItem");
              uint32_t totPacketSize = currentItem->m_packet->GetSize ();
              NS_LOG_LOGIC ("Extracted " << outItemSize << " bytes");

              uint32_t oldOffset = qsb.GetOffset ();
              uint32_t newOffset = oldOffset + newPacketSize;
              bool oldOffBit = !(oldOffset == 0);
              bool newOffBit = true;
              uint32_t oldLength = qsb.GetLength ();
              uint32_t newLength = 0;
              bool newLengthBit = true;
              newLength = totPacketSize - newPacketSize;
              if (oldLength == 0)
                {
                  newLengthBit = false;
                }
              bool lengthBit = true;
              bool oldFinBit = qsb.IsStreamFin ();
              bool newFinBit = false;

              QuicSubheader newQsbToTx = QuicSubheader::CreateStreamSubHeader (qsb.GetStreamId (),
                                                                               oldOffset, newPacketSize, oldOffBit, lengthBit, newFinBit);
              QuicSubheader newQsbToBuffer = QuicSubheader::CreateStreamSubHeader (qsb.GetStreamId (),
                                                                                   newOffset, newLength, newOffBit, newLengthBit, oldFinBit);

              Ptr<Packet> firstPartPacket = currentItem->m_packet->CreateFragment (
                0, newPacketSize);
              NS_ASSERT_MSG (firstPartPacket->GetSize () == newPacketSize,
                             "Wrong size " << firstPartPacket->GetSize ());
              firstPartPacket->AddHeader (newQsbToTx);
              firstPartPacket->Print (std::cerr);

              NS_LOG_INFO ("Split packet, putting second part back in application buffer - stream " << newQsbToBuffer.GetStreamId () << ", storing from offset " << newQsbToBuffer.GetOffset ());


              Ptr<Packet> secondPartPacket = currentItem->m_packet->CreateFragment (
                newPacketSize, newLength);
              secondPartPacket->AddHeader (newQsbToBuffer);

              Ptr<QuicSocketTxItem> toBeBuffered = CreateObject<QuicSocketTxItem> (*currentItem);
              toBeBuffered->m_packet = secondPartPacket;
              currentItem->m_packet = firstPartPacket;

              QuicSocketTxItem::MergeItems (*outItem, *currentItem);
              outItemSize += currentItem->m_packet->GetSize ();

              m_appList.push (CreateObject<QuicSocketTxScheduleItem> (scheduleItem->GetStreamId (), scheduleItem->GetOffset (), scheduleItem->GetPriority (), toBeBuffered));
              m_appSize += toBeBuffered->m_packet->GetSize ();


              NS_LOG_LOGIC ("Buffer size: " << m_appSize << " (put back " << toBeBuffered->m_packet->GetSize () << " bytes)");
              break; // at most one segment
            }
        }
    }

  NS_LOG_INFO ("Update: remaining App Size " << m_appSize << ", object size " << outItemSize);

  //Print(std::cout);

  return outItem;
}
 */

/**
 * zhiy zeng: Modify by ywj
 */
Ptr<QuicSocketTxItem>
QuicSocketTxScheduler::GetNewSegment (uint32_t numBytes, uint32_t pathId, uint64_t Q, bool isFast, bool QUpdate, uint32_t fileSize, uint8_t algo)
{

  NS_LOG_FUNCTION (this << numBytes);
  //std::cout<<"I got pathId: "<<pathId<<" and Q: "<<Q<<" filesize: "<<fileSize<<std::endl;
  bool firstSegment = true;
  Ptr<Packet> currentPacket = 0;
  Ptr<QuicSocketTxItem> currentItem = 0;
  Ptr<QuicSocketTxItem> outItem = CreateObject<QuicSocketTxItem>();
  outItem->m_isStream = true;   // Packets sent with this method are always stream packets
  outItem->m_isStream0 = false;
  outItem->m_packet = Create<Packet> ();
  uint32_t outItemSize = 0;
  uint32_t sumOfParts = 0;

  if (m_appSize > 0 and !m_appList.empty()) 
    {
      Ptr<QuicSocketTxScheduleItem> firstScheduleItem = m_appList.top();
      Ptr<QuicSocketTxItem> txItem = firstScheduleItem->GetItem();
      Ptr<Packet> packet = txItem->m_packet;
      auto packetSize = packet->GetSize();

      bool isRetx = firstScheduleItem->GetPriority() == -1;
      if (isRetx) 
        {
          NS_LOG_DEBUG ("Next packet is ReTx packet " << txItem->m_packetNumber);

          NS_LOG_DEBUG ("Returning single ReTx item for packet " << txItem->m_packetNumber);

          m_appSize -= packetSize;
          m_appList.pop();

          QuicSocketTxItem::MergeItems (*outItem, *txItem);
          outItemSize += packetSize;

          NS_LOG_INFO ("Update: remaining App Size " << m_appSize << ", object size " << outItemSize);
         
          // if (m_appSize == 0)
          //   {
          //     PrintSendTimeLog();
          //   }

          return outItem;
        }
    }

  //initial m_sendRecvData
  if (ini)
    {
      m_secondPartData.insert(m_secondPartData.end(), m_secondPartData0);
      m_secondPartData.insert(m_secondPartData.end(), m_secondPartData1);
      ini = 0;
      m_leftFileSize = fileSize;
    }

  while (m_appSize > 0 && outItemSize < numBytes)
    {
      Ptr<QuicSocketTxScheduleItem> scheduleItem;

      if (!m_secondPartData[pathId].empty())
        {
          scheduleItem = m_secondPartData[pathId].top ();
          m_secondPartData[pathId].pop ();
          completeFrame = true;
          isNewData = false;
        }
      else if (!m_appList.empty())
        {
          scheduleItem = m_appList.top ();
          m_appList.pop ();
          completeFrame = false;
          isNewData = true;
        }
      else if (!m_secondPartData[(pathId+1)%2].empty()) //if neither m_secondPartData[pathId] nor m_appList has data while m_appSize > 0, 
        {                                                 //m_secondPartData[(pathId+1)%2] should not be empty 
          scheduleItem = m_secondPartData[(pathId+1)%2].top ();
          m_secondPartData[(pathId+1)%2].pop ();
          completeFrame = true;
          isNewData = false;
        }
      else
        {
          NS_ABORT_MSG ("No enough data to send!!!");
        }
      currentItem = scheduleItem->GetItem ();
      currentPacket = currentItem->m_packet;
      //std::cout<<"--m_appsize ("<<m_appSize<<") - currentpacket ("<<currentPacket->GetSize ()<<") = "<< m_appSize - currentPacket->GetSize ()<<std::endl;
      m_appSize -= currentPacket ->GetSize ();

      

    /*       Ptr<QuicSocketTxScheduleItem> scheduleItem = m_appList.top ();
      currentItem = scheduleItem->GetItem ();
      currentPacket = currentItem->m_packet;
      int size = currentPacket->GetSize ();
      std::cout<<"--m_appsize ("<<m_appSize<<") - currentpacket ("<<currentPacket->GetSize ()<<") = "<< m_appSize - currentPacket->GetSize ()<<std::endl;
      m_appSize -= currentPacket->GetSize ();
      m_appList.pop (); */


      // if (pathId != lastUsedId) 
      //   {
      //     pathChangeFlag = 1;

      //     if (outItemSize + currentItem->m_packet->GetSize () <= numBytes)
      //       {
      //         m_secondPartData[lastUsedId].push(scheduleItem);
      //       }
      //     if (pathId == 0 and !m_secondPartData0.empty())
      //       {
      //         scheduleItem = m_secondPartData0.top ();
      //         currentItem = scheduleItem->GetItem ();
      //         currentPacket = currentItem->m_packet;
      //         m_secondPartData0.pop ();
      //       }
      //   }

      // if (outItemSize + currentItem->m_packet->GetSize ()   /*- subheaderSize*/
      //     <= numBytes )       // Merge
      
      if (outItemSize + currentItem->m_packet->GetSize ()   /*- subheaderSize*/
          <= numBytes and completeFrame)  //
        {
          NS_LOG_LOGIC ("Add complete frame to the outItem - size "
                        << currentItem->m_packet->GetSize ()
                        << " m_appSize " << m_appSize);

          QuicSubheader qsb;
          currentPacket->PeekHeader (qsb);
          NS_LOG_INFO ("Packet: stream " << qsb.GetStreamId () << ", offset " << qsb.GetOffset ());

          if (qsb.GetOffset() == 4965438)
          {
            std::cout<<"teset";
          }

          // std::cout<<"Packet: stream " << qsb.GetStreamId () << ", ----------oldoffset: " << qsb.GetOffset ()
          //           <<" qsb.GetSerializedSize ():"<<qsb.GetSerializedSize ()
          //           <<" m_frametype: "<<(uint64_t)qsb.GetFrameType()<<std::endl;
          
          QuicSocketTxItem::MergeItems (*outItem, *currentItem);
          outItemSize += currentItem->m_packet->GetSize ();

          m_offsetSendTimeInfo.push_back (std::make_pair(qsb.GetOffset(), Simulator::Now ().GetSeconds ()));


          NS_LOG_LOGIC ("Updating application buffer size: " << m_appSize);
          continue;
        }
      else if (firstSegment)  //ywj: we cannot transmit a full packet, so let's split it and update the subheaders
        {
          firstSegment = false;
          if (completeFrame and (algo == 3 || algo == 4 || algo == 5)) algo = 2; // if completeFrame enters this branch, force algo = 2 which would not overwirte 
                                        // the oldOffset of the complete frame, as opposed to our algo
          // get the currentPacket subheader
          QuicSubheader qsb;
          currentPacket->PeekHeader (qsb);

           // new packet size
          int newPacketSizeInt;
          uint32_t oldOffset;
          uint32_t newOffset;
          uint32_t oldLength;
          uint32_t newLength;
          bool oldOffBit;
          bool newOffBit;
          bool lengthBit;
          bool newLengthBit;
          bool oldFinBit;
          bool newFinBit;

          switch (algo)
          {
          case 3:
          case 4:
          case 5:
            {
             /**
             * ywj: redesign the offset to achieve out of order schedule for in-order arrival
             * get the new packet size according to customized offset
             **/

              //ywj: everytime when the Q is updated, we then find the new upper bound of offset for the faster path, 
              //also have to keep the old bound in case the actual sent amount of data on fast path is less than the estimated value
              if (QUpdate)
              {

                if (SP0 < lastBoundOnFast){
                  
                  if (holes.empty())
                  {
                    holes.push_back(std::make_pair(SP0,lastBoundOnFast));
                    // std::cout<<"--line531 hole add: sp0: "<<SP0<<" lastbound: "<<lastBoundOnFast<<std::endl;
                  }
                  else
                  {
                    bool ex = false;  //if the hole [x: lastBoundOnFast] alrease exists, no need to push again.
                    for (auto it:holes)
                    {

                      if (it.first == SP0 || it.second == lastBoundOnFast)      ///!!!!!
                      {
                        ex = true;
                        break;
                      }
                    }

                    if (!ex) 
                      holes.push_back(std::make_pair(SP0,lastBoundOnFast));
                      // std::cout<<"--line548 hole add: sp0: "<<SP0<<" lastbound: "<<lastBoundOnFast<<std::endl;
                  }
                  
                }else
                {
                  SP0 = largestSent;
                }

                // if (SP0 + Q <=lastBoundOnFast || SP1 <= SP0)  //hole is too large such that hole >= Q, or at there is no hole exists. 
                // {
                //   boundOnFast = SP0 + Q;
                // }
                if (holes.empty())
                {
                  boundOnFast = std::min(fileSize, SP0 + (uint32_t)Q);
                }
                else      // holes exist
                {
                  //boundOnFast = SP0 + Q + largestSent - boundOnFast;
                  uint32_t tempQ = Q;
                  uint32_t rightMost;
                  for (auto it:holes)                                       
                    {
                      uint32_t holeGap = it.second - it.first;
                      tempQ -= holeGap;
                      rightMost = it.second;
                    }
                  if (tempQ >= 0)
                    {
                      boundOnFast = largestSent + tempQ;
                    }
                  else
                    {
                      boundOnFast = rightMost;
                    }
                  boundOnFast = std::min(fileSize, boundOnFast);
                  if (largestSent > 0 and boundOnFast > largestSent)
                  {
                    bool ex = false;  //if the hole [x: lastBoundOnFast] alrease exists, no need to push again.
                    for (auto it:holes)
                    {
                      if (it.first == largestSent || it.second == boundOnFast)
                      {
                        ex = true;
                        if (it.second > boundOnFast)     // 
                        {
                          boundOnFast = it.second;
                        }
                        break;
                      }
                    }
                    if (!ex) 
                      {
                        holes.push_back(std::make_pair(largestSent, boundOnFast));
                        //std::cout<<"--line598 hole add: largestSent: "<<largestSent<<" boundOnFast: "<<boundOnFast<<std::endl;
                      }
                  }
                }
                std::cout<<"lastboundonfast: "<<lastBoundOnFast<<"largestSent"<<largestSent<<"boundOnFast"<<boundOnFast<<std::endl;
              }

              if (isFast) // isFast = 1 means we are scheduling for the fast path
              {
                
                if (SP0 >= fileSize){
                  SP0 = SP1; 
                  oldOffset = SP0;
                  //std::cout<<" >>>>>>>>>>oldoffsfet >= fileSize, SP1: "<<SP1<<std::endl;
                }else{
                  
                  bool firstRound = !lastBoundOnFast && SP0 < boundOnFast;  //at the first round of scheduling, no lastBoundOnFast record (i.e., lastBoundOnFast = 0)
                  bool nonFirstRound = lastBoundOnFast && SP0 < lastBoundOnFast;  //
                  
                  if (!holes.empty())
                  {
                    inHole = true;
                    std::sort(holes.begin(), holes.end());
                    auto firstHole = holes.begin ();

                    uint32_t leftBound = (*firstHole).first;

                    SP0 = leftBound;
                    oldOffset = SP0;

                    /* SP0 = largestSent;
                    oldOffset = SP0; */
                  }else if (firstRound || nonFirstRound){
                      oldOffset = SP0;
                  }else{
                    SP0 = largestSent;
                    oldOffset = SP0;
                  }

                }
              }else
              {  // we are scheduling for the slow path
                // initialize SP1

                lastBoundOnFast = boundOnFast;  //put this to here to make sure the lastbound is updated whenever the slow path is actually used
                //std::cout<<"lastbbbbb: "<<lastBoundOnFast<<std::endl;
                SP1 = std::max (boundOnFast, largestSent);
                if (SP1 >= fileSize){
                  //break;    //if SP1 exceeds the filesize, we would not allocate data to slow path and return empty.
                  //SP1 = SP0;
                  // std::cout<<" >>>>>>>>>>SP1 >= fileSize, SP1: "<<SP1
                  //           <<" filesize: "<<fileSize
                  //           <<" boundONfast: "<<boundOnFast
                  //           <<" largestSent: "<<largestSent
                  //           <<" S0: "<<SP0<<std::endl;
                  NS_ABORT_MSG ("SP1 >= fileSize!!!");
                }
                oldOffset = SP1;
              }


              oldOffBit = !(oldOffset == 0);

              oldLength = qsb.GetLength ();
              lengthBit = true;
              oldFinBit = qsb.IsStreamFin ();

              uint32_t streamId = qsb.GetStreamId ();

              uint32_t serializedSize = CalculateSubHeaderLength (oldLength, streamId, oldOffset, oldOffBit, lengthBit, oldFinBit);


              // uint32_t leftBytes = std::min (numBytes, fileSize-oldOffset); 
              // newPacketSizeInt = leftBytes - outItemSize - serializedSize;
              uint32_t remaining = int (numBytes - outItemSize - serializedSize) > 0 ? int (numBytes - outItemSize - serializedSize) : 0;
              newPacketSizeInt = std::min (remaining, fileSize-oldOffset);
    /*              if (inHole)
                {
                  auto currentHole = holes.begin();
                  newPacketSizeInt = std::min(newPacketSizeInt, (int)((*currentHole).second - (*currentHole).first));
                  inHole = false;
                } */

              // std::cout<<"----------oldoffset: "<<oldOffset<<" my serializedsize: "<<serializedSize<<" newpacketsize: "<<(uint32_t)newPacketSizeInt<<std::endl;
              // std::cout<<"--**** QuicSocketTxScheduler::GetNewSegment numBytes: "<<(int)numBytes
                      // <<" outItemSize: "
                      // <<outItemSize
                      // <<" qsb.GetSerializedSize ():"<<qsb.GetSerializedSize ()
                      // <<" m_frametype: "<<(uint64_t)qsb.GetFrameType()<<std::endl; 
            if (oldOffset == 9735280)
            {
              //std::cout<<"hhfiouhdioufhdeuifh";
            }

            }
            
            break;
          
          default:
            uint32_t leftBytes = std::min (numBytes, currentItem->m_packet->GetSize ()); 
            newPacketSizeInt = (int)leftBytes - outItemSize - qsb.GetSerializedSize ();
            break;
          }

            
          if (newPacketSizeInt <= 0)
            {
              NS_LOG_INFO ("Not enough bytes even for the header");
              m_appList.push (scheduleItem);
              m_appSize += currentPacket->GetSize ();
              break;
            }
          else
            {
              NS_LOG_INFO ("Split packet on stream " << qsb.GetStreamId () << ", sending " << newPacketSizeInt << " bytes from offset " << qsb.GetOffset ());
              // std::cout<<"Split packet on stream " << qsb.GetStreamId () << ", sending " << newPacketSizeInt << " bytes from offset " << qsb.GetOffset ()<<std::endl;

              currentPacket->RemoveHeader (qsb);
              uint32_t newPacketSize = (uint32_t)newPacketSizeInt;

              NS_LOG_LOGIC ("Add incomplete frame to the outItem");
              uint32_t totPacketSize = currentItem->m_packet->GetSize ();
              NS_LOG_LOGIC ("Extracted " << outItemSize << " bytes");


              switch (algo)
              {
              case 3:
              case 4:
              case 5:
                {
                  newOffset = oldOffset + newPacketSize;

                  if (newOffset >= fileSize) 
                    {
                      if (isFast)
                      {
                        newOffset = SP1;
                       // std::cout<<" >>>>>>>>>>newoffsfet >= fileSize, SP1: "<<SP1<<std::endl;
                      }else{
                        newOffset = SP0;
                        //std::cout<<" >>>>>>>>>>newoffsfet >= fileSize, SP0: "<<SP0<<std::endl;
                        //TODO: deal with the case that S1 reaches filesize and then point to SP0, we should consider the holes for slow path 
                      }

                    }
                  newOffBit = true;
                  newLength = 0;
                  newLengthBit = true;

                  uint32_t hole = std::min (totPacketSize - newPacketSize, fileSize - newOffset);
                  if (newOffset < boundOnFast)
                  {
                    newLength = std::min (hole, boundOnFast - newOffset); //ywj: ensure the ending offset to not exceed fileSize as well as the boundOnFast
                  }else
                  {
                    newLength = hole; //ywj: ensure the ending offset to not exceed fileSize
                  }
                  
                  if (oldLength == 0)
                    {
                      newLengthBit = false;
                    }
                  newFinBit = false;
                }
                
                break;
              
              default:
                {
                  oldOffset = qsb.GetOffset ();
                  newOffset = oldOffset + newPacketSize;
                  oldOffBit = !(oldOffset == 0);
                  newOffBit = true;
                  uint32_t oldLength = qsb.GetLength ();
                  newLength = 0;
                  newLengthBit = true;
                  newLength = totPacketSize - newPacketSize;
                  if (oldLength == 0)
                    {
                      newLengthBit = false;
                    }
                  lengthBit = true;
                  oldFinBit = qsb.IsStreamFin ();
                  newFinBit = false;
                }

                break;
              }

              QuicSubheader newQsbToTx = QuicSubheader::CreateStreamSubHeader (qsb.GetStreamId (),
                                                                               oldOffset, newPacketSize, oldOffBit, lengthBit, newFinBit);
              QuicSubheader newQsbToBuffer = QuicSubheader::CreateStreamSubHeader (qsb.GetStreamId (),
                                                                                   newOffset, newLength, newOffBit, newLengthBit, oldFinBit);
              sumOfParts = newPacketSize + newLength;
              Ptr<Packet> firstPartPacket = currentItem->m_packet->CreateFragment (
                0, newPacketSize);
              NS_ASSERT_MSG (firstPartPacket->GetSize () == newPacketSize,
                             "Wrong size " << firstPartPacket->GetSize ());
              firstPartPacket->AddHeader (newQsbToTx);
              firstPartPacket->Print (std::cerr);
              std::cout<<Simulator::Now ().GetSeconds ()
                        <<" QuicSocketTxScheduler::GetNewSegment "
                        <<" <>range: ["<<oldOffset<<" : "<<oldOffset+newPacketSize<<"] on path "<<pathId<<std::endl;
              NS_LOG_INFO ("Split packet, putting second part back in application buffer - stream " << newQsbToBuffer.GetStreamId () << ", storing from offset " << newQsbToBuffer.GetOffset ());

              Ptr<Packet> secondPartPacket = currentItem->m_packet->CreateFragment (
                newPacketSize, newLength);
              secondPartPacket->AddHeader (newQsbToBuffer);
              secondPartPacket->Print (std::cerr);
              std::cout<<Simulator::Now ().GetSeconds ()
                        <<" QuicSocketTxScheduler::GetNewSegment "
                        <<"<>range: ["<<newOffset<<" : "<<newOffset+newLength<<"] on path "<<pathId<<std::endl;

              if (newOffset == 9999540 || oldOffset == 9999540)
              {
                std::cout<<"debug!!!\n";
              }
              // record sending time of each frame. 
              m_offsetSendTimeInfo.push_back (std::make_pair(oldOffset, Simulator::Now ().GetSeconds ()));

                  //open the file for writing with the truncate-option, which means we'll clear the content before writing new data into it
              sendTimeLog.open ("sendTimeLog.txt", std::ofstream::out | std::ofstream::trunc);
              sendTimeLog << "FrameOffset\tSending Time (s)\n";
              if (!m_offsetSendTimeInfo.empty())
                {
                  // sort the pairs of vector based on the first element
                  std::sort(m_offsetSendTimeInfo.begin(), m_offsetSendTimeInfo.end(), [](const std::pair<int,int> &left, const std::pair<int,int> &right) {return left.first < right.first;});
                  for (auto osti:m_offsetSendTimeInfo)
                  {
                    sendTimeLog << std::setfill (' ') << std::setw (4) << osti.first
                    << std::setfill (' ') << std::setw (21) << osti.second <<"\n";
                  }
                }
                else
                  {
                    NS_LOG_ERROR ("Error: m_offsetSendTimeInfo is empty");
                  }
              sendTimeLog.close();

              Ptr<QuicSocketTxItem> toBeBuffered = CreateObject<QuicSocketTxItem> (*currentItem);
              toBeBuffered->m_packet = secondPartPacket;
              currentItem->m_packet = firstPartPacket;

              QuicSocketTxItem::MergeItems (*outItem, *currentItem);
              outItemSize += currentItem->m_packet->GetSize ();

               //m_appList.push (CreateObject<QuicSocketTxScheduleItem> (scheduleItem->GetStreamId (), scheduleItem->GetOffset (), scheduleItem->GetPriority (), toBeBuffered));
               //std::cout<<"--m_appsize ("<<m_appSize<<") + toBeBuffered ("<<toBeBuffered->m_packet->GetSize ()<<") = "<< m_appSize + toBeBuffered->m_packet->GetSize ()<<std::endl;
               m_appSize += toBeBuffered->m_packet->GetSize ();

               // m_leftFileSize would be passed to QuicSocketBase::SendDataPacket, which is used to determine whether freeze the slow path or not,  
               if (isNewData)
                {
                  m_leftFileSize -= sumOfParts;
                  //std::cout<<"---m_leftFileSize: "<<m_leftFileSize<<std::endl;
                }
              m_secondPartData[pathId].push (CreateObject<QuicSocketTxScheduleItem> (scheduleItem->GetStreamId (), scheduleItem->GetOffset (), scheduleItem->GetPriority (), toBeBuffered));

              NS_LOG_LOGIC ("Buffer size: " << m_appSize << " (put back " << toBeBuffered->m_packet->GetSize () << " bytes)");
              break; // at most one segment
            }
        }

    }

    // if (m_appSize == 0)
    //   {
    //     PrintSendTimeLog();
    //   }

    NS_LOG_INFO ("Update: remaining App Size " << m_appSize << ", object size " << outItemSize);

  //ywj: increment offset start point each time after scheduling packets of 'numBytes'
  if (algo == 3 || algo == 4 || algo == 5)
    {
      if (isFast) 
        {
          SP0 += sumOfParts;

          if (!holes.empty()){
            if (SP0 < holes.at(0).second)
            {
              holes.at(0).first = SP0;
              //std::cout<<"--line840 holes.at(0).first: "<<holes.at(0).first<<" is modified as SP0: "<<SP0<<std::endl;
              if (SP0 == 287617)
              {
                //std::cout<<"llll";
              }
            }else if (SP0 == holes.at(0).second)
            {
              holes.erase(holes.begin());
              //std::cout<<"--line844 holes.erase(holes.begin()) "<<std::endl;
              if (!holes.empty())  SP0 = holes.at(0).first;
              else SP0 = largestSent;
            }else
            {
              NS_ASSERT_MSG(SP0 > holes.at(0).second,"SP0 > rightBound!!!!!");
            }
          }

        }else{  
          SP1 += sumOfParts;
        }
        largestSent = std::max (SP0,SP1);

      //check ranges overlap
      uint32_t leftPayload;
      if (!holes.empty())
        {
          uint32_t totHole = 0;
          uint32_t rightMost;
          for (auto ho: holes)
            {
              uint32_t holeGap = ho.second - ho.first;
              totHole += holeGap;
              rightMost = ho.second;
            }
          leftPayload = totHole + fileSize - SP1;
          if (rightMost == fileSize) leftPayload = totHole;
        }
      else
        {
          if (SP1 == 0)   // slow path has not bee used yet
            {
              leftPayload = fileSize - SP0;
            }
          else
            {
              uint32_t x = boundOnFast > SP0 ? boundOnFast - SP0 : 0;
              leftPayload = fileSize - largestSent + x;
            }
        }
      if (m_leftFileSize != leftPayload)
        {
           std::cout<<Simulator::Now().GetSeconds()
                <<" QuicSocketTxScheduler::GetNewSegment "
                <<" m_leftFileSize: "<<m_leftFileSize
                <<" actualLeftSize: "<<leftPayload<<std::endl;
        }
      m_leftSizeOnSlowPath = fileSize - boundOnFast;
      
    }

  
  return outItem;
  
}

/**
 * zhiy zeng: Add by ywj
 */
void
QuicSocketTxScheduler::PrintSendTimeLog () 
{
  NS_LOG_FUNCTION(this);
    //open the file for writing with the truncate-option, which means we'll clear the content before writing new data into it
  sendTimeLog.open ("sendTimeLog.txt", std::ofstream::out | std::ofstream::trunc);
  sendTimeLog << "FrameOffset\tSending Time (s)\n";
  if (!m_offsetSendTimeInfo.empty())
    {
      // sort the pairs of vector based on the first element
      std::sort(m_offsetSendTimeInfo.begin(), m_offsetSendTimeInfo.end(), [](const std::pair<int,int> &left, const std::pair<int,int> &right) {return left.first < right.first;});
      for (auto osti:m_offsetSendTimeInfo)
      {
        sendTimeLog << std::setfill (' ') << std::setw (4) << osti.first
        << std::setfill (' ') << std::setw (21) << osti.second <<"\n";
      }
    }
    else
      {
        NS_LOG_ERROR ("Error: m_offsetSendTimeInfo is empty");
      }
  sendTimeLog.close();
}

uint32_t
QuicSocketTxScheduler::AppSize (void) const
{
  return m_appSize;
}

/**
 * zhiy zeng: Add by ywj
 */
uint32_t
QuicSocketTxScheduler::FileSize (void) const
{
  return m_leftFileSize;
}

uint32_t 
QuicSocketTxScheduler::SizeOnSlowPath (void) const
{
  return m_leftSizeOnSlowPath;
}

/**
 * zhiy zeng: Add by ywj
 */
uint32_t
QuicSocketTxScheduler::CalculateSubHeaderLength (uint32_t oldLength, uint32_t streamId, uint32_t oldOffset, bool oldOffBit, bool lengthBit, bool oldFinBit)
{
  uint32_t len = 8;
  uint8_t frameType = 0b00010000 | (oldOffBit << 2) | (lengthBit << 1) | (oldFinBit); 
  switch (frameType)
    {

      case 0x10:

        len += GetVarInt64Size (streamId);
        break;


      case 0x11:

        len += GetVarInt64Size (streamId);
        // The frame marks the end of the stream
        break;

      case 0x12:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldLength);
        break;

      case 0x13:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldLength);
        // The frame marks the end of the stream
        break;

      case 0x14:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldOffset);
        break;

      case 0x15:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldOffset);
        // The frame marks the end of the stream
        break;

      case 0x16:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldOffset);
        len += GetVarInt64Size (oldLength);
        break;

      case 0x17:

        len += GetVarInt64Size (streamId);
        len += GetVarInt64Size (oldOffset);
        len += GetVarInt64Size (oldLength);
        // The frame marks the end of the stream
        break;

    }


  return (len / 8);

}

//ywj
uint32_t
QuicSocketTxScheduler::GetVarInt64Size (uint64_t varInt64)
{

  //NS_LOG_FUNCTION(this);

  if (varInt64 <= 63)
    {
      return (uint32_t) 8;
    }
  else if (varInt64 <= 16383)
    {
      return (uint32_t) 16;
    }
  else if (varInt64 <= 1073741823)
    {
      return (uint32_t) 32;
    }
  else if (varInt64 <= 4611686018427387903)
    {
      return (uint32_t) 64;
    }
  else
    {
      return 0;
    }

}


}
