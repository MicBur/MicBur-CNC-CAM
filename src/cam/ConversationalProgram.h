#ifndef GEMINI_CNC_CONVERSATIONALPROGRAM_H
#define GEMINI_CNC_CONVERSATIONALPROGRAM_H

#include "ConversationalBlock.h"
#include <vector>
#include <QString>

namespace GeminiCNC::CAM {

/**
 * @brief Hurco WinMax-artiges Gesamtprogramm bestehend aus sequenziellen Bearbeitungsblöcken.
 */
class ConversationalProgram {
public:
    QString programName{"Hurco_Teil_01"};
    std::vector<ConversationalBlock> blocks;

    ConversationalProgram() = default;

    void addBlock(const ConversationalBlock& block);
    void insertBlock(size_t index, const ConversationalBlock& block);
    void removeBlock(size_t index);
    bool moveBlockUp(size_t index);
    bool moveBlockDown(size_t index);
    void duplicateBlock(size_t index);
    [[nodiscard]] int nextBlockId() const;

    // Block mit den Daten der zugehörigen Folgeblöcke (Bohrpositionen, Inseln) für die Berechnung
    [[nodiscard]] ConversationalBlock resolvedBlock(size_t index) const;

    // Ältere Bohrblöcke (ein Zyklus + eigenes Bohrbild) in Bohrungen + Bohrpositionen aufteilen
    bool upgradeDrillBlock(size_t index);
    void upgradeLegacyDrillBlocks();

    [[nodiscard]] size_t size() const { return blocks.size(); }
    [[nodiscard]] bool empty() const { return blocks.empty(); }
    [[nodiscard]] ConversationalBlock& operator[](size_t index) { return blocks[index]; }
    [[nodiscard]] const ConversationalBlock& operator[](size_t index) const { return blocks[index]; }

    /**
     * @brief Generiert den lückenlosen Gesamtwoolpath mit automatischen Werkzeugwechseln und Rückzügen.
     */
    [[nodiscard]] Toolpath generateFullToolpath(
        const QList<Core::ToolDefinition>& toolLibrary,
        const Core::BoundingBox& stockBounds,
        const Geometry::Mesh& partMesh = Geometry::Mesh()) const;

    [[nodiscard]] QJsonObject toJson() const; // Programmstand wie in der .gprog-Datei
    [[nodiscard]] bool saveToFile(const QString& filePath) const;
    [[nodiscard]] static ConversationalProgram loadFromFile(const QString& filePath);

    [[nodiscard]] static ConversationalProgram createSampleProgram();
};

} // namespace GeminiCNC::CAM

#endif // GEMINI_CNC_CONVERSATIONALPROGRAM_H
