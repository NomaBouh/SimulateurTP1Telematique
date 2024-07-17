#include "LinkLayerLow.h"

#include "LinkLayer.h"
#include "../NetworkDriver.h"
#include "../../../DataStructures/DataBuffer.h"
#include "../../../General/Configuration.h"
#include "../../../General/Logger.h"

#include <iostream>
#include <bitset>

std::unique_ptr<DataEncoderDecoder> DataEncoderDecoder::CreateEncoderDecoder(const Configuration& config)
{
    int encoderDecoderConfig = config.get(Configuration::LINK_LAYER_LOW_DATA_ENCODER_DECODER);
    if (encoderDecoderConfig == 1)
    {
        return std::make_unique<HammingDataEncoderDecoder>();
    }
    else if (encoderDecoderConfig == 2)
    {
        return std::make_unique<CRCDataEncoderDecoder>();
    }
    else
    {
        return std::make_unique<PassthroughDataEncoderDecoder>();
    }
}


DynamicDataBuffer PassthroughDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
    return data;
}

std::pair<bool, DynamicDataBuffer> PassthroughDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    return std::pair<bool, DynamicDataBuffer>(true, data);
}


//===================================================================
// Hamming Encoder decoder implementation
//===================================================================
HammingDataEncoderDecoder::HammingDataEncoderDecoder()
{
    // À faire TP1 (si Hamming demandé)
}

HammingDataEncoderDecoder::~HammingDataEncoderDecoder()
{
    // À faire TP1 (si Hamming demandé)
}

DynamicDataBuffer HammingDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
    // Créer un buffer pour stocker les données encodées.
    DynamicDataBuffer encodedData;

    // On obtient la longueur des données d'entrée.
    size_t dataLength = data.size();
    std::cout << "Encoding data of length: " << dataLength << std::endl;

    // Retourner un buffer vide si aucune donnée n'est fournie
    if (dataLength == 0) {
        std::cout << "Input data buffer is empty!" << std::endl;
        return encodedData;
    }

    // Parcourir chaque octet du buffer de données
    for (size_t i = 0; i < dataLength; ++i)
    {
        // Récupérer l'octet actuel.
        uint8_t byte = data[i];

        // Initialiser de l'octet encodé
        uint8_t encodedByte = 0;

        // Extraire les bits de données
        bool d1 = byte & 0x01;        // Bit 1 (le moins significatif)
        bool d2 = (byte >> 1) & 0x01; // Bit 2
        bool d3 = (byte >> 2) & 0x01; // Bit 3
        bool d4 = (byte >> 3) & 0x01; // Bit 4 (le plus significatif des 4 bits de données)

        std::cout << "d1: " << d1 << " d2: " << d2 << " d3: " << d3 << " d4: " << d4 << std::endl;

        // Calcul des bits de parité en utilisant les bits de données.
        bool p1 = d1 ^ d2 ^ d4; // Parité 1 (p1)
        bool p2 = d1 ^ d3 ^ d4; // Parité 2 (p1)
        bool p3 = d2 ^ d3 ^ d4; // Parité 3 (p1)

        std::cout << "p1: " << p1 << " p2: " << p2 << " p3: " << p3 << std::endl;

        // Composition de l'octet encodé avec les bits de parité et les bits de données.
        encodedByte |= (p1 << 0); // Place p1 au bit 0 
        encodedByte |= (p2 << 1); // Place p2 au bit 1
        encodedByte |= (d1 << 2); // Place d1 au bit 2
        encodedByte |= (p3 << 3); // Place p3 au bit 3
        encodedByte |= (d2 << 4); // Place d2 au bit 4
        encodedByte |= (d3 << 5); // Place d3 au bit 5
        encodedByte |= (d4 << 6); // Place d4 au bit 6 

        // Affichage de l'octet original de 8 bits et de l'octet encodé pour le débogage de 7 bits.
        std::cout << "Bits originals de 8 bits: " << std::bitset<8>(byte) << ", Bits encodes de 7 bits: " << std::bitset<7>(encodedByte) << std::endl;

        // Ajouter l'octet encodé au buffer de données encodées
        encodedData.push_back(encodedByte);
    }

    // Affiche la longueur des données encodées pour le débogage.
    std::cout << "Encoded data length: " << encodedData.size() << std::endl;
    return encodedData;
}



std::pair<bool, DynamicDataBuffer> HammingDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    // À faire TP1 (si Hamming demandé)
    return std::pair<bool, DynamicDataBuffer>(true, data);
}



//===================================================================
// CRC Encoder decoder implementation
//===================================================================
CRCDataEncoderDecoder::CRCDataEncoderDecoder()
{
    // À faire TP1 (si CRC demandé)
}

CRCDataEncoderDecoder::~CRCDataEncoderDecoder()
{
    // À faire TP1 (si CRC demandé)
}

DynamicDataBuffer CRCDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
    // À faire TP1 (si CRC demandé)
    return data;
}

std::pair<bool, DynamicDataBuffer> CRCDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    // À faire TP1 (si CRC demandé)
    return std::pair<bool, DynamicDataBuffer>(true, data);
}


//===================================================================
// Network Driver Physical layer implementation
//===================================================================
LinkLayerLow::LinkLayerLow(NetworkDriver* driver, const Configuration& config)
    : m_driver(driver)
    , m_sendingBuffer(config.get(Configuration::LINK_LAYER_LOW_SENDING_BUFFER_SIZE))
    , m_receivingBuffer(config.get(Configuration::LINK_LAYER_LOW_RECEIVING_BUFFER_SIZE))
    , m_stopReceiving(true)
    , m_stopSending(true)
{
    m_encoderDecoder = DataEncoderDecoder::CreateEncoderDecoder(config);
}

LinkLayerLow::~LinkLayerLow()
{
    stop();
    m_driver = nullptr;
}

void LinkLayerLow::start()
{
    stop();

    start_receiving();
    start_sending();
}

void LinkLayerLow::stop()
{
    stop_receiving();
    stop_sending();
}

bool LinkLayerLow::dataReceived() const
{
    return m_receivingBuffer.canRead<DynamicDataBuffer>();
}

DynamicDataBuffer LinkLayerLow::encode(const DynamicDataBuffer& data) const
{
    return m_encoderDecoder->encode(data);
}

std::pair<bool, DynamicDataBuffer> LinkLayerLow::decode(const DynamicDataBuffer& data) const
{
    return m_encoderDecoder->decode(data);
}

void LinkLayerLow::start_receiving()
{
    m_stopReceiving = false;
    m_receivingThread = std::thread(&LinkLayerLow::receiving, this);
}

void LinkLayerLow::stop_receiving()
{
    m_stopReceiving = true;
    if (m_receivingThread.joinable())
    {
        m_receivingThread.join();
    }
}

void LinkLayerLow::start_sending()
{
    m_stopSending = false;
    m_sendingThread = std::thread(&LinkLayerLow::sending, this);
}

void LinkLayerLow::stop_sending()
{
    m_stopSending = true;
    if (m_sendingThread.joinable())
    {
        m_sendingThread.join();
    }
}


void LinkLayerLow::receiving()
{
    while (!m_stopReceiving)
    {
        if (dataReceived())
        {
            DynamicDataBuffer data = m_receivingBuffer.pop<DynamicDataBuffer>();
            std::pair<bool, DynamicDataBuffer> dataBuffer = decode(data);
            if (dataBuffer.first) // Les donnees recues sont correctes et peuvent etre utilisees
            {
                Frame frame = Buffering::unpack<Frame>(dataBuffer.second);
                m_driver->getLinkLayer().receiveData(frame);
            }
            else
            {
                // Les donnees recues sont corrompues et doivent etre delaissees
                Logger log(std::cout);
                log << m_driver->getMACAddress() << " : Corrupted data received" << std::endl;
            }
        }
    }
}

void LinkLayerLow::sending()
{
    while (!m_stopSending)
    {
        if (m_driver->getLinkLayer().dataReady())
        {
            Frame dataFrame = m_driver->getLinkLayer().getNextData();
            DynamicDataBuffer buffer = encode(Buffering::pack<Frame>(dataFrame));
            sendData(buffer);
        }
    }
}

void LinkLayerLow::receiveData(const DynamicDataBuffer& data)
{
    // Si le buffer est plein, on fait juste oublier les octets recus du cable
    // Sinon, on ajoute les octets au buffer
    if (m_receivingBuffer.canWrite<DynamicDataBuffer>(data))
    {
        m_receivingBuffer.push(data);
    }
    else
    {
        Logger log(std::cout);
        log << m_driver->getMACAddress() << " : Physical reception buffer full... data discarded" << std::endl;
    }
}

void LinkLayerLow::sendData(DynamicDataBuffer data)
{
    // Envoit une suite d'octet sur le cable connecte
    m_driver->sendToCard(data);
}