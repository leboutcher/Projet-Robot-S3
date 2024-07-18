#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    ui->setupUi(this);
    ui->lcdDistance->setDigitCount(4);
    ui->lcdEnergie->setDigitCount(4);
    ui->lcdAngle->setDigitCount(4);



    // Set segment style to Flat for all QLCDNumber widgets
    ui->lcdDistance->setSegmentStyle(QLCDNumber::Flat);
    ui->lcdAngle->setSegmentStyle(QLCDNumber::Flat);
    ui->lcdEnergie->setSegmentStyle(QLCDNumber::Flat);


    // Initialisation du graphique
    ui->graph->setChart(&chart_);
    chart_.setTitle("Position du robot");
    chart_.legend()->hide();
    chart_.addSeries(&series_);

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();
    connectTextInputs();
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
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);

            // Affichage des messages Json
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));

            // Affichage des donnees dans le graph
            /*if(jsonObj.contains(JsonKey_)){
                double time = jsonObj["time"].toDouble();
                series_.append(time, jsonObj[JsonKey_].toDouble());
                // Mise en forme du graphique (non optimal)
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }
            */
            if (jsonObj.contains("distance")) {
                double distance = jsonObj["distance"].toDouble();

                // Mettre à jour le QLCDNumber
                ui->lcdDistance->display(static_cast<int>(distance));
            }

            if (jsonObj.contains("angle")) {
                double angle = jsonObj["angle"].toDouble();

                // Mettre à jour le QLCDNumber
                ui->lcdAngle->display(static_cast<int>(angle));
            }

            if (jsonObj.contains("energie")) {
                double energie = jsonObj["energie"].toDouble();

                // Mettre à jour le QLCDNumber
                ui->lcdEnergie->display(static_cast<int>(energie));
            }

            if (jsonObj.contains("position")) {
               double time = jsonObj["time"].toDouble();
               double position = jsonObj["position"].toDouble();

               time = time/1000;

               // Ajouter les données au QLineSeries pour le graphique
               series_.append(time, position);

               // Mettre à jour le graphique
               chart_.removeSeries(&series_);
               chart_.addSeries(&series_);
               chart_.createDefaultAxes();
            }

            // Fonction de reception de message (vide pour l'instant)
            msgReceived_ = msgBuffer_;
            onMessageReceived(msgReceived_);

            // Si les donnees doivent etre enregistrees
            if(record){
                writer_->write(jsonObj);
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
    // Fonction de connection du boutton Send
    connect(ui->pulseButton, SIGNAL(clicked()), this, SLOT(sendPulseStart()));
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(manageRecording(int)));
    connect(ui->pushButton_Params, SIGNAL(clicked()), this, SLOT(sendPID()));
    // aimant
    connect(ui->onMagButton, SIGNAL(clicked()), this, SLOT(onMagButtonClicked()));
    connect(ui->offMagButton, SIGNAL(clicked()), this, SLOT(offMagButtonClicked()));
    connect(ui->onSequence, SIGNAL(clicked()), this, SLOT(onSequenceClicked()));
    connect(ui->offSequence, SIGNAL(clicked()), this, SLOT(offSequenceClicked()));
}

void MainWindow::connectSpinBoxes(){
    // Fonction de connection des spin boxes
    connect(ui->DurationBox, SIGNAL(valueChanged(int)), this, SLOT(sendPulseSetting()));
    connect(ui->PWMBox, SIGNAL(valueChanged(double)), this, SLOT(sendPulseSetting()));
}

void MainWindow::connectTextInputs(){
    // Fonction de connection des entrees de texte
    connect(ui->JsonKey, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->JsonKey->text();
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

void MainWindow::changeJsonKeyValue(){
    // Fonction SLOT pour changer la valeur de la cle Json
    series_.clear();
    JsonKey_ = ui->JsonKey->text();
}
void MainWindow::sendPID(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double goal = ui->lineEdit_DesVal->text().toDouble();
    double Kp = ui->lineEdit_Kp->text().toDouble();
    double Ki = ui->lineEdit_Ki->text().toDouble();
    double Kd = ui->lineEdit_Kd->text().toDouble();
    double thresh = ui->lineEdit_Thresh->text().toDouble();
    // pour minimiser le nombre de decimales( QString::number)

    QJsonArray array = { QString::number(Kp, 'f', 2),
                         QString::number(Ki, 'f', 2),
                         QString::number(Kd, 'f', 2),
                         QString::number(thresh, 'f', 2),
                         QString::number(goal, 'f', 2)
                       };
    QJsonObject jsonObject
    {
        {"setGoal", array}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}
void MainWindow::sendPulseSetting(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double PWM_val = ui->PWMBox->value();
    int duration_val = ui->DurationBox->value();
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"pulsePWM", QString::number(PWM_val)},
        {"pulseTime", duration_val}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendPulseStart(){
    // Fonction SLOT pour envoyer la commande de pulse
    QJsonObject jsonObject
    {
        {"pulse", 1}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
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

void MainWindow::manageRecording(int stateButton){
    // Fonction SLOT pour determiner l'etat du bouton d'enregistrement
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // Fonction SLOT pour creation d'un nouveau fichier csv
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Fonction permettant d'arreter l'ecriture du CSV
    record = false;
    delete writer_;
}
void MainWindow::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    // qDebug().noquote() << "Message du Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Fonction SLOT appelee a intervalle definie dans le constructeur
    qDebug().noquote() << "*";
}


void MainWindow::onMagButtonClicked() {

    QJsonObject jsonObject
    {
        {"magOn", 1}
    };
    
    // Formatage en document JSON
    QJsonDocument doc(jsonObject);

     //Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}
void MainWindow::offMagButtonClicked() {

    QJsonObject jsonObject
    {
        {"magOff", 1}
    };
    
    // Formatage en document JSON
    QJsonDocument doc(jsonObject);

     //Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}

void MainWindow::onSequenceClicked(){

    QJsonObject jsonObject
    {
        {"sequOn", 1}
    };
    // Formatage en document JSON
    QJsonDocument doc(jsonObject);

     //Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}

void MainWindow::offSequenceClicked(){

    QJsonObject jsonObject
    {
        {"sequOff", 1}
    };

    // Formatage en document JSON
    QJsonDocument doc(jsonObject);

     //Casting en type QString
    QString strJson(doc.toJson(QJsonDocument::Compact));

    // Envoi du message
    sendMessage(strJson);
}
