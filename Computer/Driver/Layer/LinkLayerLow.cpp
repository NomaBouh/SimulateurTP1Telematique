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
    std::cout << "----- Encodeur -----" << std::endl;

    // Taille initiale du message
    uint32_t m = data.size();
    std::cout << "Taille de la donnee entree: " << m << std::endl;

    // Afficher les bits d'entree
    std::cout << "Les bits entres sont : ";
    for (uint32_t i = 0; i < m; ++i)
    {
        std::bitset<8> byte(data[i]);
        std::cout << byte << " ";
    }
    std::cout << std::endl;

    // Calcul du nombre de bits de parite necessaires
    uint32_t r = 0;
    while ((1 << r) < (m + r + 1))
    {
        r++;
    }

    std::cout << "Nombre de bits de parite necessaires : " << r << std::endl;

    // Taille totale du message encode
    uint32_t totalSize = m + r;
    DynamicDataBuffer encodedData(totalSize);

    // Insertion des bits de donnees et des bits de parite (initialement a 0)
    uint32_t dataIndex = 0;
    for (uint32_t i = 1; i <= totalSize; ++i)
    {
        if ((i & (i - 1)) == 0)
        {
            encodedData[i - 1] = 0; // Position de parite
        }
        else
        {
            encodedData[i - 1] = data[dataIndex];
            dataIndex++;
        }
    }

    // Calcul des bits de parite
    for (uint32_t i = 0; i < r; ++i)
    {
        uint32_t parityPos = (1 << i);
        uint32_t parity = 0;
        for (uint32_t j = 1; j <= totalSize; ++j)
        {
            if (j & parityPos)
            {
                parity ^= encodedData[j - 1];
            }
        }
        encodedData[parityPos - 1] = parity;
    }

    // Affichage des donnees encodees
    std::cout << "Donnees encodees : ";
    for (uint32_t i = 0; i < totalSize; ++i)
    {
        std::cout << (int)encodedData[i];
    }
    std::cout << std::endl;

    return encodedData;
}

std::pair<bool, DynamicDataBuffer> HammingDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    std::cout << "----- Decodeur -----" << std::endl;

    // Taille du message encode
    uint32_t totalSize = data.size();

    std::cout << "Message encode recu : ";
    for (uint32_t i = 0; i < totalSize; ++i)
    {
        std::cout << (int)data[i] << " ";
    }
    std::cout << std::endl;

    // Calcul du nombre de bits de parite
    uint32_t r = 0;
    while ((1 << r) < totalSize)
    {
        r++;
    }

    std::cout << "Nombre de bits de parite : " << r << std::endl;

    // Calcul des bits de parite et detection d'erreur
    uint32_t errorPosition = 0;
    for (uint32_t i = 0; i < r; ++i)
    {
        uint32_t parityPos = (1 << i);
        uint32_t parity = 0;
        for (uint32_t j = 1; j <= totalSize; ++j)
        {
            if (j & parityPos)
            {
                parity ^= data[j - 1];
            }
        }
        if (parity != 0)
        {
            errorPosition += parityPos;
        }
    }

    std::cout << "Position de l'erreur detectee : " << errorPosition << std::endl;

    // Correction d'erreur si necessaire
    DynamicDataBuffer correctedData = data; // Copie des donnees
    if (errorPosition > 0 && errorPosition <= totalSize)
    {
        correctedData[errorPosition - 1] ^= 1;
        std::cout << "Erreur corrigee a la position " << errorPosition << std::endl;
    }

    // Extraction des bits de donnees
    DynamicDataBuffer decodedData(totalSize - r);
    uint32_t dataIndex = 0;
    for (uint32_t i = 1; i <= totalSize; ++i)
    {
        if ((i & (i - 1)) != 0)
        {
            decodedData[dataIndex] = correctedData[i - 1];
            dataIndex++;
        }
    }

    // Affichage des donnees decodees
    std::cout << "Les Donnees recues sont : ";
    for (uint32_t i = 0; i < decodedData.size(); ++i)
    {
        std::bitset<8> byte(decodedData[i]);
        std::cout << byte << " ";
    }
    std::cout << std::endl;

    // Verification si une correction a ete effectuee
    bool corrected = (errorPosition > 0);

    if (corrected)
    {
        std::cout << "Test : Buffer en erreur -> Test valide" << std::endl;
    }
    else
    {
        std::cout << "Test : Buffer valide -> Test valide" << std::endl;
    }

    return std::make_pair(corrected, decodedData);
}


//===================================================================
// CRC Encoder decoder implementation
//===================================================================
CRCDataEncoderDecoder::CRCDataEncoderDecoder()
{
    // À faire TP1 (si CRC demandé)
    std::deque<bool> generateur;
    generateur = { true, false, true, true, false, true };
}

CRCDataEncoderDecoder::~CRCDataEncoderDecoder()
{
    // À faire TP1 (si CRC demandé)
}

DynamicDataBuffer CRCDataEncoderDecoder::encode(const DynamicDataBuffer& data) const
{
    // À faire TP1 (si CRC demandé)

    std::cout << "----- Encodeur -----" << std::endl;

    uint32_t num_input_bits = data.size() * 8;
    uint32_t num_redunt_bits = 0;
    std::deque<bool> buffer(num_input_bits, 0);

    std::cout << "taille de la donnee entree: " << num_input_bits << std::endl;

    // bytes -> bits
    std::cout << "les bits entrees sont : ";
    for (int i = 0; i < data.size(); i++)
    {
        std::bitset<8> byte(data[i]);

        for (int bit = 0; bit < 8; bit++)
        {

            buffer[i * 8 + bit] = byte[7 - bit];
            std::cout << buffer[i * 8 + bit];

        }
        //i nombre de byte (5) dans l'exemple

        std::cout << " ";
    }
    // ****division de buffert et du generateur ******
    std::deque<bool> dividend = buffer;
    std::deque<bool> generateur;
    generateur = { true, false, true, true, false, true };
    // on ajoute les 5 (0) au dividend car generateur.size = 6
    for (int i = 0; i < generateur.size() - 1; ++i) {
        dividend.push_back(false);
    }

    std::deque<bool> Data = dividend;
    std::cout << std::endl;
    std::cout << "Donnees: ";
    for (const auto& value : dividend) {
        std::cout << value;
    }
    std::cout << std::endl;
    std::cout << "Le Polynome pris est : ";
    for (const auto& value : generateur) {
        std::cout << value << " ";
    }
    // taille du polynome
    int taille_generateur = generateur.size();


    // division 
    for (int i = 0; i <= dividend.size() - taille_generateur; i++)
    {
        // Si le bit le plus significatif du dividende est 0, nous continuons jusqu'à ce que nous trouvions un bit 1. 
        if (!dividend[i]) {
            continue;
        }
        // Effectuer l'opération XOR pour calculer le reste 
        for (int j = 0; j < taille_generateur; j++) {
            dividend[i + j] = dividend[i + j] ^ generateur[j];
        }
    }
    // Supprimer les zéros initiaux du reste
    while (!dividend.empty() && !dividend[0]) {
        dividend.erase(dividend.begin());
    }
    // fin de la division 
    // dividend deviens notre reste
    std::cout << std::endl;
    std::cout << "Reste : ";

    for (bool& value : dividend) {
        std::cout << value;
    }

    std::cout << std::endl;
    // ajouter des 0 pour completer si le reste est plus petit que generateur.size() - 1
    if (dividend.size() < generateur.size() - 1) {
        for (size_t i = 0; i < (generateur.size() - 1 - dividend.size()); i++) {
            buffer.insert(buffer.end(), 0);
        }
    }
    // ajouter le reste a mes bits 
    buffer.insert(buffer.end(), dividend.begin(), dividend.end());
    std::cout << "Donnees transmises a l'autre ordinateur : ";

    for (bool& value : buffer) {
        std::cout << value;
    }
    std::cout << std::endl;


    // Convertion de  buffer a DynamicDataBuffer
    std::vector<uint8_t> bufferData(buffer.size() / 8 + (buffer.size() % 8 != 0 ? 1 : 0));
    for (size_t i = 0; i < buffer.size(); ++i)
    {
        if (buffer[i])
        {
            bufferData[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    return DynamicDataBuffer(bufferData.size(), bufferData.data());;
}

std::pair<bool, DynamicDataBuffer> CRCDataEncoderDecoder::decode(const DynamicDataBuffer& data) const
{
    // À faire TP1 (si CRC demandé)
 
    std::cout << "----- Decodeur -----" << std::endl;

    // Convertir data a std::deque<bool>
    std::deque<bool> buffer;
    const uint8_t* dataPtr = data.data();
    uint32_t dataSize = data.size();

    for (uint32_t i = 0; i < dataSize; ++i)
    {
        for (int j = 7; j >= 0; --j)
        {
            buffer.push_back((dataPtr[i] >> j) & 1);
        }
    }

    // Diviser buffer par le generator
    std::deque<bool> dividend = buffer;
    std::deque<bool> generateur;
    generateur = { true, false, true, true, false, true };
    int taille_generateur = generateur.size();

    // Realizer la división
    for (int i = 0; i <= dividend.size() - taille_generateur; i++)
    {
        if (!dividend[i]) {
            continue;
        }

        //  operation XOR 
        for (int j = 0; j < taille_generateur; j++) {
            dividend[i + j] = dividend[i + j] ^ generateur[j];
        }
    }

    // verifier si le reste est zero
    bool isValid = std::all_of(dividend.begin(), dividend.end(), [](bool bit) { return !bit; });
    if (isValid == true) {
        std::cout << "Le message a ete bien recu " << std::endl;
    }
    else {
        std::cout << "Il y'a des erreurs dans les donnees " << std::endl;
    }

    // Extraire les données d'origine en supprimant le reste
    DynamicDataBuffer originalData;
    originalData = DynamicDataBuffer(dataSize - 1, dataPtr);

    std::cout << "Les Donnees recues sont : " << std::endl;
    for (size_t i = 0; i < originalData.size(); ++i)
    {
        uint8_t byte = originalData.read<uint8_t>(i);
        for (int j = 7; j >= 0; --j)
        {
            std::cout << ((byte >> j) & 1);
        }
        std::cout << " ";
    }
    std::cout << std::endl;
    return std::pair<bool, DynamicDataBuffer>(isValid, data);
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