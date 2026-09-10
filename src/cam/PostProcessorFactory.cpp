#include "PostProcessor.h"
#include "HurcoPostProcessor.h"
#include "HeidenhainPostProcessor.h"

namespace GeminiCNC::CAM {

std::unique_ptr<PostProcessor> PostProcessorFactory::create(ControllerType type) {
    switch (type) {
        case ControllerType::Hurco_WinMax:
            return std::make_unique<HurcoPostProcessor>();

        case ControllerType::Heidenhain:
            return std::make_unique<HeidenhainPostProcessor>();

        case ControllerType::Klipper:
            // Klipper nutzt ISO-Standard mit Erweiterungen
            // Später: eigene KlipperPostProcessor-Klasse
            return std::make_unique<HurcoPostProcessor>(); // Fallback

        case ControllerType::Saeilo:
            // Saeilo / Mach3 ist nahe ISO-Standard
            // TODO: eigene SaeiloPostProcessor-Klasse
            return std::make_unique<HeidenhainPostProcessor>(); // Fallback ISO

        case ControllerType::ISO_Standard:
        default:
            // Generischer ISO-PP = Basisklasse mit Default-Hooks
            // Wir erzeugen eine anonyme Subklasse inline
            class IsoPostProcessor final : public PostProcessor {
            public:
                [[nodiscard]] QString name() const override {
                    return QStringLiteral("ISO 6983 Standard");
                }
                [[nodiscard]] QString fileExtension() const override {
                    return QStringLiteral(".nc");
                }
            };
            return std::make_unique<IsoPostProcessor>();
    }
}

} // namespace GeminiCNC::CAM
