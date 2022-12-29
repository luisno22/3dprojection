//**************************************************************************//
//*                            Luis Nolasco Ramirez                        *//
//* g++ calibration.cpp -o calibration `pkg-config opencv --cflags --libs` *//
//**************************************************************************//
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#define WIDTH 25 // Anchura de la casilla del tablero en mm
#define HEIGHT 25 // Altura de la casilla del tablero en mm

using namespace std;
using namespace cv;

//***********DECLARACION DE FUNCIONES***********//

int gestionarArgs(int, char* [], bool&, string&, bool&);
void calibrarCamara(Mat&, Mat&, bool);
void reconstruccion3D(Mat, Mat, bool, string);
void dibujarPrisma(Mat, vector<Point2f>);
void dibujarFigura(Mat, Mat, Mat, Mat, vector<Point2f>, Mat, float);

//VARIABLES GLOBLALES//
Size boardSize(6,9); // Tamaño del tablero NxM (filas,columnas)

//**********************//
//*********MAIN*********//
//**********************//



int main(int argc, char* argv[])
{
    bool mostrarResultado = false;
    bool streamMode = false;
    string videoName;
    int mode = gestionarArgs(argc,argv,mostrarResultado,videoName,streamMode);

    Mat distortionCoefficients;
    Mat cameraMatrix;
    
    // Realiza las funciones correspondietnes al modo indicado resultado de los argumentos
    if(mode == 1){ // solo calibracion
        calibrarCamara(cameraMatrix,distortionCoefficients,mostrarResultado);
    } else if (mode == 2){ // solo reconstruccion 3d
        FileStorage fs( "parameters.xml", FileStorage::READ );
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distortionCoefficients;
        fs.release();
        reconstruccion3D(cameraMatrix,distortionCoefficients,streamMode,videoName);
    } else if (mode == 3){ // calibracion seguida de reconstruccion 3d
        calibrarCamara(cameraMatrix,distortionCoefficients,mostrarResultado);
        reconstruccion3D(cameraMatrix,distortionCoefficients,streamMode,videoName);
    } else if(mode < 0){
        
    } else {
        cerr << "ERROR: se necesitan argumentos principales [-c] [-r]" << endl << "Uso:" << endl << "practica1 [-c] -r file [-s] [-h]" << endl;
    }
    
    return 0;
}


/**
  @brief Recibe los argumentos de entrada y los asigna a sus variables correspondientes con el tipo de dato necesario
  @param argc numero de argumentos
  @param argv array que almacena todos los argumentos
  @param mostrarResultado mostrar o no el resultado de la busqueda de esquina en la imagen
  @param videoName en caso de indicarse un video pregrabado el nombre del video se almacena en esta variable
  @param streamMode Se realiza la reconstruccion 3d con un video en directo o con un video pregrabado

  @returns Modo de ejecucon del codigo: 1 calibrar camara, 2 reconstruccion 3d de un video, 3 calibrado de camara y reconstruccion 3d
  */
int gestionarArgs(int argc, char* argv[], bool &mostrarResultado, string &videoName, bool &streamMode)
{
    int mode = 0;
    if(argc == 1) {
        cerr << "ERROR: se necesitan argumentos" << endl << "Uso:" << endl << "practica1 [-c] -r file [-s] [-h]" << endl;
        exit(EXIT_FAILURE);
    } else { 
        for(int i = 1; i < argc; i++) {
            if (strcmp(argv[i],"-c") == 0) {
                mode += 1;
            } else if (strcmp(argv[i],"-r") == 0) {
                mode += 2;
                if(!ifstream(argv[i+1])) {
                    streamMode = true;
                    cout << "** La reconstruccion 3D se hara a partir de la imagen en directo de la camara predeterminada **" << endl;
                } else { 
                    videoName = argv[i+1];
                    cout << "** La reconstruccion 3D se llevara a cabo a paritr del video " << videoName << " **" << endl;
                    i++;
                }
            } else if(strcmp(argv[i],"-s") == 0) {
                mostrarResultado = true;
            } else if (strcmp(argv[i],"-h") == 0) {
                mode -= 100;
                cout << "Usage:" << endl << "practica1 [-c] -r file [-s] [-h]" << endl;
                cout << "\t -- [-c] : Calibrar camara. * Se deben colocar las imagenes que se quieran usar para la calibración dentro de una carpeta 'images' en el directorio. Los parametros de la camara se guardaran en el fichero 'parameters.xml' para su posterior lectura en reconstruccion 3D. *" << endl;
                cout << "\t -- [-r] : Reconstruccion 3D. * Se lee el fichero 'parameters.xml' para obtener los parametros intrinsecos de la camara. Si se indica un archivo video despues de este argumento se creara la reconstruccion en ese video en lugar de en la captura en directo de la camara. *" << endl; 
                cout << "\t -- [-s] : Mostrar esquinas. * Si se usa este argumento, se mostrarán las esquinas encontradas en cada imagen. *" << endl;
                cout << "\t -- [-h] : Muestra la ayuda y obvia los demas argumentos " << endl;
            } else {
                cerr << "ERROR: opcion desconocida " << argv[i] << endl << "Uso:" << endl << "practica1 [-c] -r file [-s] [-h]" << endl;
                exit(EXIT_FAILURE);
            } 
        }   
        return mode;
    }
}

/**
  @brief Calibra la camara
  @param cameraMatrix matriz con los parametros de la camara
  @param distortionCoefficientes matriz con los coeficientes de distorsion
  @param mostrarResultado mostrar o no el resultado de la busqueda de esquina en la imagen

  @returns Escribe un archivo xml con a matriz de la camara y los coeficientes de distorsion
  */
void calibrarCamara(Mat &cameraMatrix, Mat &distortionCoefficients, bool mostrarResultado)
{
    vector <Mat> images; // Vector que almacena las imagenes
    vector< vector<Point2f> > imgPoints; // Vector que almacena los keypoints de la imagen en el sistema de referencia de la camara
    vector< vector<Point3f> > objPoints; // Vector que almacena los puntos del tablero en el sistema de referencia del tablero
    vector<Point3f> objP;
    vector<Point2f> corners;
    vector<Mat> imagesGray;

    for( int y = 0; y < boardSize.height; y++ )  // Se definen los puntos del objeto con sistema de referencia en el propio tablero
    {
        for( int x = 0; x < boardSize.width; x++ )
        {
            objP.push_back(cv::Point3f(float(x*WIDTH), float(y*HEIGHT), 0));
        }
    }

    // ** Obteniendo las imagenes del tablero para la calibracion ** //
    // Lee todas las imagenes que se encuentren en la carpeta donde se ejecuta el codigo siguiendo los numeros enteros 
    // de forma ascendente hasta la ultima imagen que continua de forma ininterrumpida dicha serie
    int i=0;
    bool end=false;
    do{
        i++;
        string imageFileName = "images/" + to_string(i) +".jpeg";
        Mat image = imread(imageFileName,1);
        if(!image.data){
            end=true;
        }else{
            images.push_back(image);
        }
        
    }while(!end);


    Size imageSize = images.at(0).size();


    // Encuentra las esquinas del tablero para todas las imagenes
    imagesGray.resize(images.size());

    for (int i=0; i<images.size(); i++)
    {
        // Busca las intersecciones del tablero de calibracion y las almacena en el vector corners
        bool patternFound = findChessboardCorners(images.at(i), boardSize, corners);

        //Si encuentra un patron en la imagen anterior se evalua
        if (patternFound) 
        {
            // Mejora y reifna la precisión de las coordenadas de las esquinas
            cvtColor(images.at(i), imagesGray.at(i), CV_RGB2GRAY);
            cornerSubPix(imagesGray.at(i), corners, Size(3,2), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
            
            // Muestras las imagenes de calibracion con las esquinas dibujadas en caso de recibir el argumento correspondiente
            if(mostrarResultado)
            {
                drawChessboardCorners(images.at(i), boardSize, Mat(corners), patternFound );
                imshow("Corners find", images.at(i));
                int keyPressed;
                do
                {
                    keyPressed = waitKey(0);
                } while (keyPressed != 'q');
            }

            // Guarda las esquinas del patron en sus respectivos vectores para sus respectivos sistemas de referencia
            imgPoints.push_back(corners);
            objPoints.push_back(objP);
        }
    }

    // Parametros de la camara, INICIALIZACION
    distortionCoefficients = Mat::zeros(8, 1, CV_64F);
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    vector<Mat> rotationVectors;
    vector<Mat> translationVectors;
    
    // Obtencion los parametros intrinsecos de la camara
    calibrateCamera(objPoints, imgPoints, imageSize, cameraMatrix,
                    distortionCoefficients, rotationVectors, translationVectors);

    // GUARDANDO DATOS EN FICHERO XML

    FileStorage fs( "parameters.xml", FileStorage::WRITE );
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distortionCoefficients;
    fs.release();
    
    //--OBTENCION DE LOS PARAMETROS DE LA CAMARA FINALIZADA--//
    cout << "** Calibración de la camara terminada **" << endl;
    cout << "** Información guardada en archivo xml **" <<endl;
    //-------------------------------------------------------//
    
}

/**
  @brief Reconstruccion 3d a partir de un video en directo captado por la camara por defecto del dispositivo en el que se ejecute el codigo
  @param cameraMatrix matriz con los parametros de la camara
  @param distortionCoefficientes matriz con los coeficientes de distorsion
  @param videoName en caso de indicarse un video pregrabado el nombre del video se almacena en esta variable
  @param streamMode Se realiza la reconstruccion 3d con un video en directo o con un video pregrabado

  @returns Segraba un archivo de video con el nombre "output.avi" que almacena el video con la reconstruccion
  */
void reconstruccion3D(Mat cameraMatrix, Mat distortionCoefficients, bool streamMode, string videoName)
{
    cout << "** Iniciando reconstruccion 3D **" <<endl;
    //---------------------------------------------------------------------//
    VideoCapture cap;
    if(streamMode){
        cap.open(0); 
    }else{
        cap.open(videoName);
    }
    
    if (!cap.isOpened()) {
        cerr << "ERROR: Unable to open camera" << endl;
        exit(EXIT_FAILURE);
    }
    vector<Point2f> puntosReconst;
    vector<int> inlier;
    vector<Point2f> framePoints; // Vector que almacena los keypoints del frame en el sistema de referencia de la camara
    vector<Point3f> objPoints_frame; // Vector que almacena los puntos del tablero en el sistema de referencia del tablero
    vector<Point3f> objP_frame;
    vector<Point2f> corners_frame;
    Mat frameGray;

    // Parametros extrinsecos
    Mat rvec; // Vector de rotacion
    Mat tvec; // Vector de transalacion
    for( int y = 0; y < boardSize.height; y++ )  // Se definen los puntos del objeto con sistema de referencia en el propio tablero
    {
        for( int x = 0; x < boardSize.width; x++ )
        {
            objP_frame.push_back(cv::Point3f(float(x*WIDTH),float(y*HEIGHT),0));
        }
    }


    // Size del frame
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    // Crea un archivo de video donde almacena el resultado
    VideoWriter video("outputVideo.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
    
    cout << "** Pulsa ESC para finalizar..." <<endl;
    while(1) // Bucle infinito hasta que se acabe el video o se cierre la camara
    {
        Mat frame;
        cap >> frame;
        
        if(frame.empty())
            break;
        
        // Busca las intersecciones del tablero de calibracion y las almacena en el vector corners
        bool patternFound_frame = findChessboardCorners(frame, boardSize, corners_frame); 
        
        //Si encuentra un patron en la imagen anterior se evalua
        if(patternFound_frame)
        {
            // Mejora y reifna la precisión de las coordenadas de las esquinas
            cvtColor(frame, frameGray, CV_RGB2GRAY);
            cornerSubPix(frameGray, corners_frame, Size(3,2), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

            framePoints = corners_frame;
            objPoints_frame = objP_frame;

            //Calcula los parametros extrinsecos de la camara para el frame actual (rvec y tvec)
            bool solve = solvePnP(objPoints_frame,framePoints,cameraMatrix,distortionCoefficients,rvec,tvec);

            //Factor de escala, multiplicando esta variable se puede escalar la figura completa
            float t = ((float) WIDTH); // Ej. escala x2: t = ((float) WIDTH) * 2;
            
            dibujarFigura(rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst,frame,t);
        }
        
        flip(frame,frame,1); // Se da la vuelta al frame para que actue como un espejo
        video.write(frame);
        
        imshow("linea", frame);
        
        char c =(char)waitKey(1);
        if(c==27)
            break;
    }
    
    cap.release();
    destroyAllWindows();
    cout<<"** Reconstruccion finalizada, archivo guardado **"<<endl;
}

/**
  @brief Dibuja un prisma dados los vertices y la imagen
  @param frame un unico frame de video
  @param puntosReconst vector de puntos que contiene los vertices del prisma

  @returns
  */
void dibujarPrisma(Mat frame, vector<Point2f> puntosReconst)
{
            //ABAJO
            line(frame,puntosReconst[0],puntosReconst[1],Scalar (0,255,0),3,LINE_8);
            line(frame,puntosReconst[1],puntosReconst[2],Scalar (0,255,0),3,LINE_8);
            line(frame,puntosReconst[2],puntosReconst[3],Scalar (0,255,0),3,LINE_8);
            line(frame,puntosReconst[3],puntosReconst[0],Scalar (0,255,0),3,LINE_8);
            //ARRIBA
            line(frame,puntosReconst[4],puntosReconst[5],Scalar (255,0,0),3,LINE_8);
            line(frame,puntosReconst[5],puntosReconst[6],Scalar (255,0,0),3,LINE_8);
            line(frame,puntosReconst[6],puntosReconst[7],Scalar (255,0,0),3,LINE_8);
            line(frame,puntosReconst[7],puntosReconst[4],Scalar (255,0,0),3,LINE_8);
            //COLUMNAS
            line(frame,puntosReconst[0],puntosReconst[4],Scalar (0,0,255),3,LINE_8);
            line(frame,puntosReconst[1],puntosReconst[5],Scalar (0,0,255),3,LINE_8);
            line(frame,puntosReconst[2],puntosReconst[6],Scalar (0,0,255),3,LINE_8);
            line(frame,puntosReconst[3],puntosReconst[7],Scalar (0,0,255),3,LINE_8);
    return;
}

/**
  @brief Dibuja una figura previamente establecidos sus puntos en posicion relativa al tablero de ajedrez
  @param rvec
  @param tvec
  @param cameraMatrix matriz de la camara
  @param distortionCoefficients coeficientes de distorsion
  @param puntosReconst coordenadas vertices de la figura con respecto al tablero
  @param frame un unico frame
  @param t longitud de las aristas que forman la figura

  @returns
  */
void dibujarFigura(Mat rvec,Mat tvec, Mat cameraMatrix, Mat distortionCoefficients, vector<Point2f> puntosReconst, Mat frame, float t)
{
    // La figura esta compuesta de diversos prismas.
    // Cada prisma se define de manera individual indicando las coordenadas de cada uno de sus cuatro vertices //

    //Puntos pata1: {0,0,0},{1*t,0,0},{1*t,1*t,0},{0,1*t,0},{0,0,-2*t},{1*t,0,-2*t},{1*t,1*t,-2*t},{0,1*t,-2*t}
    vector<Point3f> pata1{{1*t,2*t,0},{2*t,2*t,0},{2*t,3*t,0},{1*t,3*t,0},{1*t,2*t,-2*t},{2*t,2*t,-2*t},{2*t,3*t,-2*t},{1*t,3*t,-2*t}};
    //Puntos pata2: {3*t,0,0},{4*t,0,0},{4*t,1*t,0},{3*t,1*t,0},{3*t,0,-2*t},{4*t,0,-2*t},{4*t,1*t,-2*t},{3*t,1*t,-2*t}
    vector<Point3f> pata2{{4*t,2*t,0},{5*t,2*t,0},{5*t,3*t,0},{4*t,3*t,0},{4*t,2*t,-2*t},{5*t,2*t,-2*t},{5*t,3*t,-2*t},{4*t,3*t,-2*t}};
    //Puntos pata3: {0,4*t,0},{1*t,4*t,0},{1*t,5*t,0},{0,5*t,0},{0,4*t,-2*t},{5*t,4*t,-2*t},{5*t,5*t,-2*t},{0,5*t,-2*t}
    vector<Point3f> pata3{{1*t,6*t,0},{2*t,6*t,0},{2*t,7*t,0},{1*t,7*t,0},{1*t,6*t,-2*t},{2*t,6*t,-2*t},{2*t,7*t,-2*t},{1*t,7*t,-2*t}};
    //Puntos pata4: {3*t,4*t,0},{4*t,4*t,0},{4*t,5*t,0},{3*t,5*t,0},{3*t,4*t,-2*t},{4*t,4*t,-2*t},{4*t,5*t,-2*t},{3*t,5*t,-2*t}
    vector<Point3f> pata4{{4*t,6*t,0},{5*t,6*t,0},{5*t,7*t,0},{4*t,7*t,0},{4*t,6*t,-2*t},{5*t,6*t,-2*t},{5*t,7*t,-2*t},{4*t,7*t,-2*t}};
    //Puntos cuerpo: {0,0,-2*t},{4*t,0,-2*t},{4*t,5*t,-2*t},{0,5*t,-2*t},   {0,0,-4*t},{4*t,0,-4*t},{4*t,5*t,-4*t},{0,5*t,-4*t}
    vector<Point3f> cuerpo{{1*t,2*t,-2*t},{5*t,2*t,-2*t},{5*t,7*t,-2*t},{1*t,7*t,-2*t},   {1*t,2*t,-4*t},{5*t,2*t,-4*t},{5*t,7*t,-4*t},{1*t,7*t,-4*t}};
    //Puntos cuello: {t,5*t,-4*t},{3*t,5*t,-4*t},{3*t,4*t,-4*t},{t,4*t,-4*t},   {t,1*t+5*t,-6*t},{3*t,1*t+5*t,-6*t},{3*t,1*t+4*t,-6*t},{t,1*t+4*t,-6*t}
    vector<Point3f> cuello{{2*t,7*t,-4*t},{4*t,7*t,-4*t},{4*t,6*t,-4*t},{2*t,6*t,-4*t},   {2*t,1*t+7*t,-6*t},{4*t,1*t+7*t,-6*t},{4*t,1*t+6*t,-6*t},{2*t,1*t+6*t,-6*t}};
    //Puntos cabeza: {t,5*t,-6*t},{t,7*t,-6*t},{3*t,7*t,-6*t},{3*t,5*t,-6*t}, {t,5*t,-8*t},{t,7*t,-8*t},{3*t,7*t,-8*t},{3*t,5*t,-8*t}
    vector<Point3f> cabeza{{2*t,7*t,-6*t},{2*t,9*t,-6*t},{4*t,9*t,-6*t},{4*t,7*t,-6*t}, {2*t,7*t,-8*t},{2*t,9*t,-8*t},{4*t,9*t,-8*t},{4*t,7*t,-8*t}};

    //Pata 1
    projectPoints(pata1,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Pata 2
    projectPoints(pata2,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Pata 3
    projectPoints(pata3,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Pata 4
    projectPoints(pata4,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Cuerpo
    projectPoints(cuerpo,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Cuello
    projectPoints(cuello,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
    // Cabeza
    projectPoints(cabeza,rvec,tvec,cameraMatrix,distortionCoefficients,puntosReconst);
    dibujarPrisma(frame,puntosReconst);
}



