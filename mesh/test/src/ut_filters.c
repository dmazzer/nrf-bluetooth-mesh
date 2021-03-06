/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmock.h>
#include <unity.h>

#include "scanner.h"
#include "filter_engine.h"
#include "gap_address_filter.h"
#include "ad_type_filter.h"
#include "rssi_filter.h"
#include "adv_packet_filter.h"

typedef enum
{
    ACCEPTED_COUNTER,
    INVALID_LENGTH_COUNTER,
    FILTERED_AD_TYPE_COUNTER,
    FILTERED_ADV_TYPE_COUNTER,
    FILTERED_GAP_ADDR_COUNTER,
    FILTERED_RSSI_COUNTER
} statistic_counter_t;

static uint32_t m_packets_accepted;
static uint32_t m_packets_dropped_invalid_addr;
static uint32_t m_packets_dropped_invalid_rssi;
static uint32_t m_packets_dropped_invalid_adtype;
static uint32_t m_packets_dropped_invalid_adv_type;

static packet_t test_packet = {.header = {BLE_PACKET_TYPE_ADV_NONCONN_IND, 0, 1, 0, 37, 0},
                               .addr   = {1, 2, 3, 4, 5, 6},
                               .payload = {0x0}};

void setUp(void)
{
    test_packet.payload[0] = 30;
    test_packet.payload[1] = AD_TYPE_MESH;
}

void tearDown(void)
{}

static void all_ad_types_add(void)
{
    for (uint16_t ad_type = 0; ad_type < 256; ad_type++)
    {
        bearer_adtype_add(ad_type);
    }
}

static void all_ble_packet_types_add(void)
{
    for (uint16_t ble_type = 0; ble_type < 16; ble_type++)
    {
        bearer_adv_packet_add(ble_type);
    }
}

static inline void statistic_check(statistic_counter_t counter_type, uint32_t counter)
{
    char * message = NULL;
    uint32_t read_counter;

    switch (counter_type)
    {
        case INVALID_LENGTH_COUNTER:
            message = "Length";
            read_counter = bearer_invalid_length_amount_get();
            break;
        case FILTERED_AD_TYPE_COUNTER:
            message = "AD type";
            read_counter = bearer_adtype_filtered_amount_get();
            break;
        case FILTERED_ADV_TYPE_COUNTER:
            message = "Adv type";
            read_counter = bearer_adv_packet_filtered_amount_get();
            break;
        case FILTERED_GAP_ADDR_COUNTER:
            message = "Addr";
            read_counter = bearer_gap_addr_filtered_amount_get();
            break;
        case FILTERED_RSSI_COUNTER:
            message = "Rssi";
            read_counter = bearer_rssi_filtered_amount_get();
            break;
        case ACCEPTED_COUNTER:
            message = "Accepted";
            read_counter = fen_accepted_amount_get();
            break;
        default:
            TEST_FAIL();
            TEST_FAIL_MESSAGE("Unacceptable counter type");
            return;
    }

    TEST_ASSERT_EQUAL_MESSAGE(counter, read_counter, message);
}

void test_gap_address_filtering(void)
{
    scanner_packet_t scanner_packet;
    packet_t * p_packet = &scanner_packet.packet;

    memcpy(p_packet, &test_packet, sizeof(packet_t));

    ble_gap_addr_t filter_list[3];
    filter_list[0].addr_type = 1;
    memset(filter_list[0].addr, 0xAB, BLE_GAP_ADDR_LEN);
    filter_list[1].addr_type = 0;
    memset(filter_list[1].addr, 0xCD, BLE_GAP_ADDR_LEN);
    filter_list[2].addr_type = 2; /* invalid addr type */
    memset(filter_list[2].addr, 0xEF, BLE_GAP_ADDR_LEN);

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, bearer_filter_gap_addr_whitelist_set(NULL, 3));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, bearer_filter_gap_addr_whitelist_set(filter_list, 0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_whitelist_set(filter_list, 3));

    memcpy(p_packet->addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[0].addr_type;

    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    memcpy(p_packet->addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[1].addr_type;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Doesn't match invalid AD type in filter */
    memcpy(p_packet->addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = 1;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    /* Not in whitelist */
    memset(p_packet->addr, 0x01, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[0].addr_type;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    /* Alter list to make previous packet pass */
    memset(filter_list[0].addr, 0x01, BLE_GAP_ADDR_LEN);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Test clear */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_blacklist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_clear());

    /* Blacklist tests */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_blacklist_set(filter_list, 3));

    memcpy(p_packet->addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[0].addr_type;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    memcpy(p_packet->addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[1].addr_type;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    /* Doesn't match invalid AD type in filter */
    memcpy(p_packet->addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = 1;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Not in blacklist */
    memset(p_packet->addr, 0x02, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[0].addr_type;;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Alter list to make previous packet fit */
    memset(filter_list[0].addr, 0x02, BLE_GAP_ADDR_LEN);
    filter_list[1].addr_type = 1;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    /* Test clear */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_whitelist_set(filter_list, 3));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, bearer_filter_gap_addr_clear());

    /* Test that all packets pass when not running filters */
    memcpy(p_packet->addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[0].addr_type;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    memcpy(p_packet->addr, filter_list[1].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = filter_list[1].addr_type;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Doesn't match invalid AD type in filter */
    memcpy(p_packet->addr, filter_list[2].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = 1;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Not in blacklist */
    memset(p_packet->addr, 0x02, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = 1;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* test range filter */

    memset(filter_list[0].addr, 0xAB, BLE_GAP_ADDR_LEN);
    memset(filter_list[1].addr, 0xAB, BLE_GAP_ADDR_LEN);
    filter_list[0].addr[5] = 0;
    filter_list[1].addr[5] = 0;
    filter_list[0].addr_type = 0;
    filter_list[1].addr_type = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list)); /* Should be allowed to set it twice */
    filter_list[1].addr[5] = 1;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));
    filter_list[0].addr[5] = 2; /* first higher than second */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, bearer_filter_gap_addr_range_set(filter_list));
    filter_list[0].addr[5] = 0;
    filter_list[0].addr_type = 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, bearer_filter_gap_addr_range_set(filter_list));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, bearer_filter_gap_addr_range_set(NULL));
    filter_list[0].addr_type = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_range_set(filter_list));

    memcpy(p_packet->addr, filter_list[0].addr, BLE_GAP_ADDR_LEN);
    p_packet->header.addr_type = 0;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    p_packet->header.addr_type = 1;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    p_packet->header.addr_type = 0;
    p_packet->addr[5] = 2;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    p_packet->addr[5] = 0;
    p_packet->addr[4] = 0;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    p_packet->addr[5] = 0;
    p_packet->addr[4] = 0xAC;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    p_packet->addr[5] = 1;
    p_packet->addr[4] = 0xAB;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_addr++;
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    filter_list[1].addr[5] = 2;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);
    statistic_check(FILTERED_GAP_ADDR_COUNTER, m_packets_dropped_invalid_addr);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_filter_gap_addr_clear());
}

void test_adtype_filtering(void)
{
    scanner_packet_t scanner_packet;
    packet_t * p_packet = &scanner_packet.packet;

    memcpy(p_packet, &test_packet, sizeof(packet_t));

    /* AD type filtering not enabled */
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* whitelist mode */
    /* AD type filtering enabled with empty filters*/
    bearer_adtype_filtering_set(true);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Disable Ad type filtering */
    bearer_adtype_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable Ad type filtering  and add the AD type in use*/
    bearer_adtype_filtering_set(true);
    bearer_adtype_add(AD_TYPE_MESH);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Use an AD type that's not been added */
    p_packet->payload[1] = AD_TYPE_BEACON;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Add the new AD type */
    bearer_adtype_add(AD_TYPE_BEACON);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Remove the unused AD type */
    bearer_adtype_remove(AD_TYPE_MESH);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Use the removed AD type */
    p_packet->payload[1] = AD_TYPE_MESH;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Disable AD type filtering */
    bearer_adtype_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add all possible AD types to the filters and enable AD type filtering */
    all_ad_types_add();
    bearer_adtype_filtering_set(true);
    for (uint16_t i = 0; i < 256; i++)
    {
        p_packet->payload[1] = i;
        TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
        m_packets_accepted++;
    }
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Remove from the filters and use AD_TYPE_MESH. */
    p_packet->payload[1] = AD_TYPE_MESH;
    bearer_adtype_remove(AD_TYPE_MESH);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Using any other type should work since only AD_TYPE_MESH is removed*/
    p_packet->payload[1] = AD_TYPE_DFU;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* clear all AD types */
    bearer_adtype_clear();
    for (uint16_t i = 0; i < 256; i++)
    {
        p_packet->payload[1] = i;
        TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
        m_packets_dropped_invalid_adtype++;
    }
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* switch off AD type filter */
    bearer_adtype_filtering_set(false);

    /* blacklist mode */
    bearer_adtype_mode_set(AD_FILTER_BLACKLIST_MODE);
    /* AD type filtering enabled with empty filters*/
    bearer_adtype_filtering_set(true);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Disable Ad type filtering */
    bearer_adtype_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable Ad type filtering  and add the AD type in use*/
    bearer_adtype_filtering_set(true);
    p_packet->payload[1] = AD_TYPE_MESH;
    bearer_adtype_add(AD_TYPE_MESH);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Use an AD type that's not been added */
    p_packet->payload[1] = AD_TYPE_BEACON;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add the new AD type */
    bearer_adtype_add(AD_TYPE_BEACON);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Remove the unused AD type */
    bearer_adtype_remove(AD_TYPE_MESH);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Use the removed AD type */
    p_packet->payload[1] = AD_TYPE_MESH;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Disable AD type filtering */
    bearer_adtype_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add all possible AD types to the filters and enable AD type filtering */
    all_ad_types_add();
    bearer_adtype_filtering_set(true);
    for (uint16_t i = 0; i < 256; i++)
    {
        p_packet->payload[1] = i;
        TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
        m_packets_dropped_invalid_adtype++;
    }
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Remove from the filters and use AD_TYPE_MESH. */
    p_packet->payload[1] = AD_TYPE_MESH;
    bearer_adtype_remove(AD_TYPE_MESH);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Using any other type should work since only AD_TYPE_MESH is removed*/
    p_packet->payload[1] = AD_TYPE_DFU;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adtype++;
    statistic_check(FILTERED_AD_TYPE_COUNTER, m_packets_dropped_invalid_adtype);

    /* Clear all AD types */
    bearer_adtype_clear();
    for (uint16_t i = 0; i < 256; i++)
    {
        p_packet->payload[1] = i;
        TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
        m_packets_accepted++;
    }
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* switch off AD type filter */
    bearer_adtype_filtering_set(false);
}

void test_rssi_filtering(void)
{
    scanner_packet_t scanner_packet =
    {
        .metadata.rssi = -5
    };
    packet_t * p_packet = &scanner_packet.packet;

    memcpy(p_packet, &test_packet, sizeof(packet_t));

    /* Rssi filtering not enabled */
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable rssi filtering. Set level lower than in the packet */
    bearer_rssi_filtering_set(-7);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable rssi filtering. Set level higher than in the packet */
    bearer_rssi_filtering_set(-3);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_rssi++;
    statistic_check(FILTERED_RSSI_COUNTER, m_packets_dropped_invalid_rssi);

    /* Disable rssi filtering. */
    bearer_rssi_filtering_set(0);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);
    statistic_check(FILTERED_RSSI_COUNTER, m_packets_dropped_invalid_rssi);
}

void test_adv_packet_filtering(void)
{
    scanner_packet_t scanner_packet;
    packet_t * p_packet = &scanner_packet.packet;

    memcpy(p_packet, &test_packet, sizeof(packet_t));
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;

    /* BLE advertisement packet filtering was not enabled */
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* whitelist mode */
    /* BLE advertisement packet filtering enabled with empty filters*/
    bearer_adv_packet_filtering_set(true);
    bearer_adv_packet_filter_mode_set(ADV_FILTER_WHITELIST_MODE);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Disable BLE advertisement packet filtering */
    bearer_adv_packet_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable BLE advertisement packet filtering and add the packet type in use*/
    bearer_adv_packet_filtering_set(true);
    bearer_adv_packet_add(BLE_PACKET_TYPE_ADV_DIRECT_IND);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Use BLE packet type that's not been added */
    p_packet->header.type = BLE_PACKET_TYPE_SCAN_REQ;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Add the new BLE packet type */
    bearer_adv_packet_add(BLE_PACKET_TYPE_SCAN_REQ);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Remove the unused BLE packet type */
    bearer_adv_packet_remove(BLE_PACKET_TYPE_ADV_DIRECT_IND);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Use the removed BLE packet type */
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Use the main for BT mesh BLE packet type */
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Disable BLE packet type filtering */
    bearer_adv_packet_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add all possible BLE packet types to the filters and enable filtering */
    all_ble_packet_types_add();
    bearer_adv_packet_filtering_set(true);
    for (uint8_t i = 0; i < 16; i++)
    {
        p_packet->header.type = i;
        TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
        m_packets_accepted++;
    }
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Remove from the filters and use BLE_PACKET_TYPE_ADV_DIRECT_IND. */
    bearer_adv_packet_remove(BLE_PACKET_TYPE_ADV_DIRECT_IND);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Using any other type should work since only BLE_PACKET_TYPE_ADV_DIRECT_IND is removed*/
    p_packet->header.type = BLE_PACKET_TYPE_SCAN_REQ;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* clear all BLE packet types */
    bearer_adv_packet_clear();
    for (uint8_t i = 0; i < 16; i++)
    {
        p_packet->header.type = i;
        if (p_packet->header.type == BLE_PACKET_TYPE_ADV_NONCONN_IND)
        {
            TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
            m_packets_accepted++;
        }
        else
        {
            TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
            m_packets_dropped_invalid_adv_type++;
        }
    }
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* switch off BLE packet type filter */
    bearer_adv_packet_filtering_set(false);

    /* blacklist mode */
    bearer_adv_packet_filter_mode_set(ADV_FILTER_BLACKLIST_MODE);
    /* BLE packet type filtering enabled with empty filters*/
    bearer_adv_packet_filtering_set(true);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Disable BLE packet type filtering */
    bearer_adv_packet_filtering_set(false);
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Enable BLE packet type filtering  and add the type in use*/
    bearer_adv_packet_filtering_set(true);
    p_packet->header.type = BLE_PACKET_TYPE_SCAN_REQ;
    bearer_adv_packet_add(BLE_PACKET_TYPE_SCAN_REQ);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Use an BLE packet type that's not been added */
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add the new BLE packet type */
    bearer_adv_packet_add(BLE_PACKET_TYPE_ADV_DIRECT_IND);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Remove the unused BLE packet type */
    bearer_adv_packet_remove(BLE_PACKET_TYPE_SCAN_REQ);
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Use the removed AD type */
    p_packet->header.type = BLE_PACKET_TYPE_SCAN_REQ;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Adding BLE_PACKET_TYPE_ADV_NONCONN_IND to the blacklist does not have effect */
    bearer_adv_packet_add(BLE_PACKET_TYPE_ADV_NONCONN_IND);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Disable BLE packet type filtering */
    bearer_adv_packet_filtering_set(false);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Add all possible BLE packet types to the filters and enable filtering */
    all_ble_packet_types_add();
    bearer_adv_packet_filtering_set(true);
    for (uint8_t i = 0; i < 16; i++)
    {
        p_packet->header.type = i;
        if (p_packet->header.type == BLE_PACKET_TYPE_ADV_NONCONN_IND)
        {
            TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
            m_packets_accepted++;
        }
        else
        {
            TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
            m_packets_dropped_invalid_adv_type++;
        }
    }
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Remove from the filters and use BLE_PACKET_TYPE_ADV_DIRECT_IND. */
    bearer_adv_packet_remove(BLE_PACKET_TYPE_ADV_DIRECT_IND);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_DIRECT_IND;
    TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
    m_packets_accepted++;
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);

    /* Using any other type should not work since only BLE_PACKET_TYPE_ADV_DIRECT_IND is removed*/
    p_packet->header.type = BLE_PACKET_TYPE_SCAN_REQ;
    TEST_ASSERT_TRUE(fen_filters_apply(&scanner_packet));
    m_packets_dropped_invalid_adv_type++;
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* Clear all BLE packet types */
    bearer_adv_packet_clear();
    for (uint8_t i = 0; i < 16; i++)
    {
        p_packet->header.type = i;
        TEST_ASSERT_FALSE(fen_filters_apply(&scanner_packet));
        m_packets_accepted++;
    }
    statistic_check(ACCEPTED_COUNTER, m_packets_accepted);
    statistic_check(FILTERED_ADV_TYPE_COUNTER, m_packets_dropped_invalid_adv_type);

    /* switch off BLE packet type filter */
    bearer_adv_packet_filtering_set(false);
}
