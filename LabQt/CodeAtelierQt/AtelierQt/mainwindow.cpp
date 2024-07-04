#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QJsonObject>
#include <QTextEdit>

#define ON_DURATION 2

MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    ui->setupUi(this);
    this->setWindowTitle("Atelier Qt");

    // Initialisation du graphique
    // Étape 2.2: Associer chart_ au QChartView dans l'interface
    // TODO...
    ui->graphicsView->setChart(&chart_);
    chart_.setTitle("Etat lumiere");
    chart_.legend()->hide();
    chart_.addSeries(&series_);

    // Étape 2.3: Donner un titre au graphique
    // TODO...

    // Étape 2.4: Cacher la légende
    // TODO...

    // Étape 2.5: Associer series_ à chart_
    // TODO...

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectComboBox();

    // Recensement des ports
    portCensus();

    // initialisation du timer
    updateTimer_.start();
}

MainWindow::~MainWindow(){
    // Destructeur de la classe
    updateTimer_.stop();
    if(serialCom_!=nullptr){
      delete serialCom_;
    }
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Fonction appelee lorsque la fenetre est detruite
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg){
    // Fonction appelee lors de reception sur port serie
    // Accumulation des morceaux de message
    msgBuffer_ += msg;

    //Si un message est termine
    if(msgBuffer_.endsWith('\n')){
        // Passage ASCII vers structure Json
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer_.toUtf8());

        // Analyse du message Json
        if(~jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();

            if(jsonObj.contains("lightState")) {
                double time = jsonObj["time"].toDouble();
                double lightState = jsonObj["lightState"].toDouble();

                // Étape 3. Ajouter les données à series_ et mettre à jour chart_
                // TODO...
                series_.append(time, lightState);
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }

            else {
                msgReceived_ = msgBuffer_;
                onMessageReceived(msgReceived_);
            }
        }
        // Reinitialisation du message tampon
        msgBuffer_ = "";
    }
}

void MainWindow::connectTimers(int updateRate){
    // Fonction de connection de timers
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead(){
    // Fonction de connection au message de la classe (serialProtocol)
    connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    /*
     * Étape 4.1: Connecter le bouton pour allumer la lumière
     * ---
     * widget1: ui->lightButton
     * signal: clicked()
     * widget2: this
     * slot: onLightButtonClicked()
    */
    // TODOSLO
    connect(ui->lightButton, SIGNAL(clicked()), this, SLOT(onLightButtonClicked()));

    /*
     * Étape 4.2: Connecter le bouton pour envoyer un message
     * ---
     * widget1: ui->msgButton
     * signal: clicked()
     * widget2: this
     * slot: onMsgButtonClicked()
    */
    // TODO
    connect(ui->msgButton, SIGNAL(clicked()), this, SLOT(onMsgButtonClicked()));
}

void MainWindow::onLightButtonClicked() {
    // Commenter au besoin
    qDebug().noquote() <<"Bouton lumière";

    /*
     * Étape 5. Créer un objet QJsonObject contenant la paire: "turnOnLight" et la durée d'allumage en secondes.
     * Décommenter le reste des lignes de la fonction ensuite. Ces lignes se chargent de formater
     * le message et de le transmettre par le port série.
    */
    // TODO...
    
    QJsonObject jsonObject
    {
        {"turnOnLight", 3}
    };
    
    // Formatage en document JSON
    QJsonDocument doc(jsonObject);

     //Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}

void MainWindow::onMsgButtonClicked() {
    // Commenter au besoin
    qDebug().noquote() <<"Bouton message";

    /*
     * Étape 6. Créer un objet QJsonObject contenant la paire: "usrMsg" et le texte de msgEditor.
     * Décommenter le reste des lignes de la fonction ensuite. Ces lignes se chargent de formater
     * le message et de le transmettre par le port série.
    */
    // TODO...

    QJsonObject jsonObject2
    {
        {"usrMsg", ui->msgEditor->toPlainText()}
    };

    // Formatage en document JSON
    QJsonDocument doc(jsonObject2);

    // Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}

void MainWindow::connectComboBox(){
    // Fonction de connection des entrees deroulantes
    connect(ui->comboBoxPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}

void MainWindow::portCensus(){
    // Fonction pour recenser les ports disponibles
    ui->comboBoxPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(port.portName());
    }
}

void MainWindow::startSerialCom(QString portName){
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();
}


void MainWindow::sendMessage(QString msg){
    // Fonction SLOT d'ecriture sur le port serie
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Erreur aucun port serie !!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs){
    // Fonction d'initialisation du chronometre
    updateTimer_.start(rateMs);
}

void MainWindow::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    qDebug().noquote() << "Message du Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Fonction SLOT appelee a intervalle definie dans le constructeur
    qDebug().noquote() << "*";
}
