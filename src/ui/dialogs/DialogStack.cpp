#include "DialogStack.h"

namespace GeminiCNC::UI {

DialogStack::DialogStack(QWidget* parent) : QWidget(parent) {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // Seitentitel (schmale Leiste oben)
    m_lblTitle = new QLabel(this);
    m_lblTitle->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_lblTitle->setFixedHeight(28);
    m_lblTitle->setStyleSheet(QStringLiteral(
        "background-color: #2D333B; color: #D1D5DB; font-weight: bold; font-size: 12px; "
        "padding-left: 10px; border-bottom: 1px solid #374151;"));
    mainLayout->addWidget(m_lblTitle);

    // Seiten-Container
    m_stackedWidget = new QStackedWidget(this);
    m_stackedWidget->setStyleSheet(QStringLiteral("background-color: #1B1F23;"));
    mainLayout->addWidget(m_stackedWidget, 1);
}

void DialogStack::addPage(DialogPage page, QWidget* widget, const QString& title) {
    m_stackedWidget->insertWidget(static_cast<int>(page), widget);
    while (m_pageTitles.size() <= static_cast<int>(page)) {
        m_pageTitles.append(QString());
    }
    m_pageTitles[static_cast<int>(page)] = title;
}

void DialogStack::setCurrentPage(DialogPage page) {
    int idx = static_cast<int>(page);
    if (idx >= 0 && idx < m_stackedWidget->count()) {
        m_stackedWidget->setCurrentIndex(idx);

        QString title = (idx < m_pageTitles.size()) ? m_pageTitles[idx] : QStringLiteral("Dialog");
        m_lblTitle->setText(title);

        emit pageChanged(page);
    }
}

DialogPage DialogStack::currentPage() const {
    return static_cast<DialogPage>(m_stackedWidget->currentIndex());
}

} // namespace GeminiCNC::UI
