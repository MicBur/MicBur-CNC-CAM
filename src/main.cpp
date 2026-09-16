#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include <QColor>
#include <QFont>
#include <QIcon>
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("Gemini CNC Control"));
    app.setApplicationVersion(QStringLiteral("1.1.0"));
    app.setApplicationDisplayName(QStringLiteral("MicBur-CNC-CAM"));
    app.setWindowIcon(QIcon(QStringLiteral(":/icons/app_icon.png")));
    app.setOrganizationName(QStringLiteral("Gemini Anti-Gravity"));

    // Modernes Fusion Dark Styling für Industrie-Software
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(26, 32, 44));
    darkPalette.setColor(QPalette::WindowText, QColor(247, 250, 252));
    darkPalette.setColor(QPalette::Base, QColor(15, 23, 42));
    darkPalette.setColor(QPalette::AlternateBase, QColor(30, 41, 59));
    darkPalette.setColor(QPalette::ToolTipBase, QColor(255, 255, 255));
    darkPalette.setColor(QPalette::ToolTipText, QColor(26, 32, 44));
    darkPalette.setColor(QPalette::Text, QColor(247, 250, 252));
    darkPalette.setColor(QPalette::Button, QColor(45, 55, 72));
    darkPalette.setColor(QPalette::ButtonText, QColor(247, 250, 252));
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(66, 153, 225));
    darkPalette.setColor(QPalette::Highlight, QColor(49, 130, 206));
    darkPalette.setColor(QPalette::HighlightedText, Qt::white);
    darkPalette.setColor(QPalette::Disabled, QPalette::Text, QColor(113, 128, 150));
    darkPalette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(113, 128, 150));

    app.setPalette(darkPalette);

    QFont font("Segoe UI", 9);
    app.setFont(font);

    GeminiCNC::UI::MainWindow window;
    window.show();

    return app.exec();
}
