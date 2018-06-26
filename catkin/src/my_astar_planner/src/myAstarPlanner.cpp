/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

  list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);
  bool isContains(list<coupleOfCells> & list1, int cellID);

  MyastarPlanner::MyastarPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      //world_model_ = new base_local_planner::CostmapModel(*costmap_);

      //el plan se va a publicar en el topic "planTotal"
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
      //los puntos del espacio de búsqueda se visualizan en "visualization_marker"
      marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
      marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
      marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);


      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

//@@
  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    //double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    //return footprint_cost;

  	geometry_msgs::Point punto;
    unsigned int coord_x, coord_y;
    int max_danger = 0;

    for (int i = 0; i < footprint.size(); i++) {
      punto = footprint[i];
      costmap_->worldToMap(x_i + punto.x, y_i + punto.y, coord_x, coord_y);

      if (coord_x >= 0 && coord_x < costmap_->getSizeInCellsX() && coord_y >= 0 && coord_y < costmap_->getSizeInCellsY()) {
        unsigned char cost = costmap_->getCost(coord_x, coord_y);

        if (cost == costmap_2d::LETHAL_OBSTACLE)
          return 99999;

          else{ //Comprobar Rango
                  /*
            for(int i=-5; i<=5; i++)
              for(int j=-5; j<=5; j++){

                if( i != 0 || j != 0){
                  cost = costmap_->getCost(coord_x+i, coord_y+j);

                  if(cost == costmap_2d::LETHAL_OBSTACLE && max_danger < 10 - abs(i+j)){
                    max_danger = 10 - abs(i+j);
                }

              }

          }
                  */
            for(int i=-2; i<=2; i++)
              for(int j=-2; j<=2; j++){

                if( i != 0 || j != 0){
                  cost = costmap_->getCost(coord_x+i, coord_y+j);

                  if(cost == costmap_2d::LETHAL_OBSTACLE)
                    return 99999;
                
                }

              }
          }
                
        }
    }

	return 0;
  }


  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    //***********************************************************
    // Inicio de gestion de ROS
    //***********************************************************

    unsigned t1,t2;

    double tiempo;


    t1 = clock();

    if(!initialized_){
      ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    closedList.clear();
    openList.clear();
    closedList2.clear();
    openList2.clear();

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();


    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a estructura coupleOfCells
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent =cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);

    // ##
     cpstart.fCost = cpstart.hCost;

    //insertamos la casilla inicial en abiertos
    MyastarPlanner::openList.push_back(cpstart);
    ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    ROS_INFO("Index del goal: %d", cpgoal.index );

    //*************
    // Para la búsqueda bidireccional 
    //*************

    coupleOfCells cpstart2, cpgoal2;
    double goal_x2 = start.pose.position.x;
    double goal_y2 = start.pose.position.y;
    unsigned int mgoal_x2, mgoal_y2;
    costmap_->worldToMap(goal_x2,goal_y2,mgoal_x2, mgoal_y2);
    cpgoal2.index = MyastarPlanner::costmap_->getIndex(mgoal_x2, mgoal_y2);
    cpgoal2.parent=0;
    cpgoal2.gCost=0;
    cpgoal2.hCost=0;
    cpgoal2.fCost=0;

    double start_x2 = goal.pose.position.x;
    double start_y2 = goal.pose.position.y;
    unsigned int mstart_x2, mstart_y2;
    costmap_->worldToMap(start_x2,start_y2, mstart_x2, mstart_y2);
    cpstart2.index = MyastarPlanner::costmap_->getIndex(mstart_x2, mstart_y2);
    cpstart2.parent = cpstart.index;
    cpstart2.gCost = 0;
    cpstart2.hCost = MyastarPlanner::calculateHCost(cpstart2.index,cpgoal2.index);

    // ##
    cpstart2.fCost = cpstart2.hCost;

    MyastarPlanner::openList2.push_back(cpstart2);


    /**************************************************************************/
    /*************** GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    //visualization_msgs::Marker points;// definida en la clase como markers_OpenList
    inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.0f);
    inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
    inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);

    limpiaMarkers(marker_Open_publisher, markers_ClosedList);
    limpiaMarkers(marker_Closed_publisher, markers_OpenList);

    /**************************************************************************/
    /*************** FIN GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    //visualizamos start.
    visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);

    unsigned int explorados = 0;
    unsigned int currentIndex = cpstart.index;
    unsigned int currentIndex2 = cpstart2.index;
    double distancia_final = 0;
    int distancia_nodos = 0;

    bool FOUND = false;
    list<coupleOfCells>::iterator l1, l2;
    coupleOfCells CP;
    coupleOfCells COfCells,COfCells2;

    int rango_generacion = 1, maximo_nodos =3000;
    bool cambio=true, bidireccional = true;  

    while (!MyastarPlanner::openList.empty()) //while the open list is not empty continuie the search
    {
      
      // Busco una casilla que ambas búsquedas tengan en abiertos
      for(list<coupleOfCells>::iterator i=openList.begin(); i != openList.end(); i++)
        for( list<coupleOfCells>::iterator j=openList2.begin(); j != openList2.end(); j++){

          if(i->index == j->index){
            FOUND = true;
            l1 = i;
            l2 = j;
          }
        }

      // Si lo encuentro y estoy en búsqueda bidireccional, 
      // creo un camino uniendo ambos
    
      if(FOUND && bidireccional){

        createWay( l1, l2, CP, distancia_final, currentIndex, cpgoal.index, COfCells );

      }else{
    
        // Compruebo explorados para cambiar de búsqueda
        if(explorados > maximo_nodos && cambio){

          bidireccional = false;
          rango_generacion = 3;

          openList.clear();
          closedList.clear();

          openList.push_back(cpstart);
          cambio = false;

        }

        // Saco primera celda de openList

        COfCells= openList.front();
        openList.pop_front();
        currentIndex=COfCells.index;

        COfCells2= openList2.front();
        openList2.pop_front();
        currentIndex2=COfCells2.index;

        //obtenemos un iterador al elemento en la lista de abiertos
        //list<coupleOfCells>::iterator it=getPositionInList(openList,currentIndex);
        //y la insertamos en cerrados
        MyastarPlanner::closedList.push_back(COfCells);
        MyastarPlanner::closedList2.push_back(COfCells2);

      }

        visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells.index);
        visualizaCelda(marker_Closed_publisher, markers_ClosedList, COfCells2.index);


        if(!bidireccional)
          // Compruebo distancia para reducir rango a 1
          checkDistance(cpgoal, COfCells, rango_generacion, currentIndex);

          // if the currentCell is the goalCell: success: path found

          if(currentIndex==cpgoal.index)
          {

            distancia_final = COfCells.gCost;
            //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
            //vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
             ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
              //ros::Duration(10).sleep();
            //convertimos goal a poseStamped nueva
              geometry_msgs::PoseStamped pose;
              pose.header.stamp =  ros::Time::now();
              pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
              pose.pose.position.x = goal_x;
              pose.pose.position.y = goal_y;
              pose.pose.position.z = 0.0;
              pose.pose.orientation.x = 0.0;
              pose.pose.orientation.y = 0.0;
              pose.pose.orientation.z = 0.0;
              pose.pose.orientation.w = 1.0;

              //lo añadimos al plan
              plan.push_back(pose);


              coupleOfCells currentCouple = cpgoal;
              unsigned int currentParent = COfCells.parent;
              ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
              //ros::Duration(1).sleep();

              while (currentParent != cpstart.index) //e.d. mientras no lleguemos al nodo start
              {
                //encontramos la posición de currentParent en cerrados
                list<coupleOfCells>::iterator it=getPositionInList(closedList,currentParent);

                //hacemos esa posición que sea el currentCouple
                coupleOfCells currentCouple;
                currentCouple.index=currentParent;
                currentCouple.parent=(*it).parent;
                currentCouple.gCost=(*it).gCost;
                currentCouple.hCost=(*it).hCost;
                currentCouple.fCost=(*it).fCost;

                //creamos una PoseStamped con la informaciuón de currentCouple.index

                //primero hay que convertir el currentCouple.index a world coordinates
                unsigned int mpose_x, mpose_y;
                double wpose_x, wpose_y;

                costmap_->indexToCells((*it).index, mpose_x, mpose_y);
                costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

                ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x,mpose_y,wpose_x, wpose_y, (*it).parent);
                //ros::Duration(1).sleep();

                //después creamos la pose
                geometry_msgs::PoseStamped pose;
                pose.header.stamp =  ros::Time::now();
                pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
                pose.pose.position.x = wpose_x;
                pose.pose.position.y = wpose_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                //insertamos la pose en el plan
                plan.push_back(pose);
                ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
                //hacemos que currentParent sea el parent de currentCouple
                currentParent = (*it).parent;
                distancia_nodos++;
              }

            ROS_INFO("Sale del bucle de generación del plan.");
            std::reverse(plan.begin(),plan.end());

            //lo publica en el topic "planTotal"
            //publishPlan(plan);
            t2 = clock();
            tiempo = (double (t2-t1)/CLOCKS_PER_SEC);

            cout<<endl<<endl<<"-----------------"<<endl;
            ROS_INFO("TIEMPO: %f",tiempo);
            cout<<endl;
            cout<<"NODOS EXPANDIDOS: "<<explorados<<endl;
            cout<<"DISTANCIA REAL: "<<distancia_final<<endl;
            cout<<"DISTANCIA NODOS: "<<distancia_nodos<<endl;

            return true;
          }


          /* #################################

           		Aqui empieza la búsqueda 

          ################################## */ 

          //search the neighbors of the current Cell
          vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex, rango_generacion);
          vector <unsigned int> neighborCells2=findFreeNeighborCell(currentIndex2, rango_generacion);
          ROS_INFO("Ha encontrado %u vecinos", (unsigned int)neighborCells.size());

          //neighbors that exist in the closedList are ignored
          vector <unsigned int> neighborNotInClosedList;
          vector <unsigned int> neighborNotInClosedList2;

          for(uint i=0; i<neighborCells.size(); i++)
          {
            if(!isContains(closedList,neighborCells[i]))
            {
              neighborNotInClosedList.push_back(neighborCells[i]);
            }
          }
          ROS_INFO("Ha encontrado %u vecinos que no están en cerrados", (unsigned int)neighborNotInClosedList.size());
          for(uint i=0; i<neighborCells2.size(); i++)
          {
            if(!isContains(closedList2,neighborCells2[i]))
            {
              neighborNotInClosedList2.push_back(neighborCells2[i]);
            }
          }

          //search the neighbors that already exist in the open List
          vector <unsigned int> neighborsInOpenList;
          vector <unsigned int> neighborsNotInOpenList;
          vector <unsigned int> neighborsInOpenList2;
          vector <unsigned int> neighborsNotInOpenList2;

          for(uint i=0; i<neighborNotInClosedList.size(); i++)
          {
            if(isContains(openList,neighborNotInClosedList[i]))
              neighborsInOpenList.push_back(neighborNotInClosedList[i]);
            else
              neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
          }

          for(uint i=0; i<neighborNotInClosedList2.size(); i++)
          {
            if(isContains(openList2,neighborNotInClosedList2[i]))
              neighborsInOpenList2.push_back(neighborNotInClosedList2[i]);
            else
              neighborsNotInOpenList2.push_back(neighborNotInClosedList2[i]);
          }


         //add the neighbors that are not in the open list to the open list and mark the current cell as their parent

         //addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, cpstart.gCost, cpgoal.index); //,tBreak);
         addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, COfCells.gCost, cpgoal.index);
         addNeighborCellsToOpenList(openList2, neighborsNotInOpenList2, currentIndex2, COfCells2.gCost, cpgoal2.index);

         explorados+=2;

         //PINTO ABIERTOS
         //Anyadir neighborCells a points. pushback()
         visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
         visualizaCelda(marker_Closed_publisher,markers_ClosedList, COfCells.index);

         visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList2);
         visualizaCelda(marker_Closed_publisher,markers_ClosedList, COfCells2.index);

         //Para los nodos que ya están en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario

         
        updateNeighborsInOpenList(openList, neighborsInOpenList, COfCells.gCost, cpgoal.index, currentIndex);
        updateNeighborsInOpenList(openList2, neighborsInOpenList2, COfCells2.gCost, cpgoal2.index, currentIndex2);
  
    }

    if(openList.empty())  // if the openList is empty: then failure to find a path
        {
            ROS_INFO("Failure to find a path !");
            return false;
           // exit(1);
        }


};

void MyastarPlanner::checkDistance(coupleOfCells &cpgoal, coupleOfCells &COfCells, int &rango_generacion, unsigned int &currentIndex){

  unsigned int mgoal_x, mgoal_y, mactual_x, mactual_y;
  double wgoal_x, wgoal_y, wactual_x, wactual_y; 

  costmap_->indexToCells(cpgoal.index, mgoal_x, mgoal_y);
  costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

  costmap_->indexToCells(COfCells.index, mactual_x, mactual_y);
  costmap_->mapToWorld(mactual_x, mactual_y, wactual_x, wactual_y);

  if(abs(wactual_x - wgoal_x) < rango_generacion && abs(wactual_y - wgoal_y) < rango_generacion){

    rango_generacion = 1;
  }

}

void MyastarPlanner::createWay(list<coupleOfCells>::iterator l1, list<coupleOfCells>::iterator l2,
          coupleOfCells &CP, double &distancia_final, unsigned int &currentIndex, unsigned int goalindex,
          coupleOfCells &COfCells ){

        closedList.push_back(*l1);
        distancia_final = l1->gCost + l2->gCost;
        l2 = getPositionInList(closedList2, l2->parent);

        do{
          
          CP.index = l2 -> index;
          CP.parent = l1 -> index;
          
          CP.gCost = 0; //l1->gCost + getMoveCost(CP.parent, CP.index);
          CP.hCost = 0; //getMoveCost(CP.index, cpgoal.index);

          CP.fCost = 0; //0.3*CP.gCost + 0.7*CP.hCost;

          closedList.push_back(CP);

          l1 = l2;

          l2 = getPositionInList(closedList2, l2->parent);

        }while( l1->index != goalindex);

         //closedList.push_back(*l1);

         COfCells = CP;
         currentIndex = CP.index;
         COfCells.gCost = distancia_final;

}

void MyastarPlanner::updateNeighborsInOpenList(list<coupleOfCells> & openList, const vector<unsigned int> &neighborsInOpenList, double costP, unsigned int indice_goal, unsigned int currentIndex){

  vector<unsigned int> neighborsInOpenListToUpdate;

  for(uint i=0; i<neighborsInOpenList.size(); i++){

    list<coupleOfCells>::iterator a = getPositionInList(openList, neighborsInOpenList[i]);

    double coste = getMoveCost(currentIndex, neighborsInOpenList[i]);

    if(a->gCost > (costP+coste)){

      neighborsInOpenListToUpdate.push_back(neighborsInOpenList[i]);
      openList.erase(a);
    }
  }

  addNeighborCellsToOpenList(openList, neighborsInOpenListToUpdate, currentIndex, costP, indice_goal);
}

//calculamos H como la distancia euclídea hasta el goal
double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
  unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
  double wstart_x, wstart_y, wgoal_x, wgoal_y;

  //trasformamos el indice de celdas a coordenadas del mundo.
  //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

  costmap_->indexToCells(start, mstart_x, mstart_y);
  costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
  costmap_->indexToCells(goal, mgoal_x, mgoal_y);
  costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

  //return ( abs(wstart_x - wgoal_x) + abs(wstart_y - wgoal_y)); //MANHATTAN

  return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2)); // EUCLIDEA

 }


//comparamos F para dos nodos.
bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2)
{
   return c1.fCost < c2.fCost;
}

/*******************************************************************************/
//Function Name: getPositnionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID)
{
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return it;

   }
}


 /*******************************************************************************
 * Function Name: findFreeNeighborCell
  * Inputs: the row and columun of the current Cell
  * Output: a vector of free neighbor cells of the current cell
  * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
  * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
//@@
vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID, int rango_generacion){

        unsigned int mx, my;
        double  wx, wy;
        bool ok;
        costmap_->indexToCells(CellID,mx,my);
        ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);

        vector <unsigned int>  freeNeighborCells;

        int rango_neg = rango_generacion*-1;

        for (int x=rango_neg;x<=rango_generacion;x+=rango_generacion)
          for (int y=rango_neg; y<=rango_generacion;y+=rango_generacion){
            //check whether the index is valid
            //ROS_INFO("A ver: X = %u, Size_X = %u, Y = %u Size_Y = %u",mx+x, (unsigned int)costmap_->getSizeInCellsX(),my+y, (unsigned int)costmap_->getSizeInCellsY());
           if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
               costmap_->mapToWorld( (unsigned int) mx+x, (unsigned int) my+y, wx, wy);

               //ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
              if(costmap_->getCost(mx+x,my+y) < 127  && (!(x==0 && y==0)) && footprintCost(wx,wy,0) != 9999){

                ok = true;

                if(x>0 && y>0){
                for(int i=1; i < x; i++)
                  for(int j=1; j < y; j++)
                    if(costmap_->getCost(mx+i,my+j) < 127)
                      ok = false;
                }else if(x>0 && y<0){
                for(int i=1; i < x; i++)
                  for(int j=-1; j > y; j--)
                    if(costmap_->getCost(mx+i,my+j) < 127)
                      ok = false;
                }else if(x<0 && y>0){
                for(int i=-1; i > x; i--)
                  for(int j=1; j < y; j++)
                    if(costmap_->getCost(mx+i,my+j) < 127)
                      ok = false;  
                }else{
                  for(int i=-1; i > x; i--)
                  for(int j=-1; j > y; j--)
                    if(costmap_->getCost(mx+i,my+j) < 127)
                      ok = false;
                }

                  if(ok){
                    unsigned int index = costmap_->getIndex(mx+x,my+y);
                     // ROS_INFO("Vecina (%f, %f)", wx,wy);
                    freeNeighborCells.push_back(index);
                  }
                  
              }
          }

      }
      return  freeNeighborCells;
      }


/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
 bool isContains(list<coupleOfCells> & list1, int cellID)
 {
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return true;
  }
   return false;
}

double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
  //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
  return calculateHCost(here,there);

}

/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output:
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void MyastarPlanner::addNeighborCellsToOpenList(list<coupleOfCells> & OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) //,float tBreak)
{
        vector <coupleOfCells> neighborsCellsOrdered;
        
        for(uint i=0; i< neighborCells.size(); i++)
        {
          coupleOfCells CP;
          CP.index=neighborCells[i]; //insert the neighbor cell
          CP.parent=parent; //insert the parent cell
          
          // ## AddOns
          CP.gCost = gCostParent + getMoveCost(parent, neighborCells[i]);;
          CP.hCost = getMoveCost(CP.index, goalCell);


          CP.fCost = 0.3*CP.gCost + 0.7*CP.hCost;

          
          OPL.insert(upper_bound(OPL.begin(),OPL.end(),CP,compareFCost),CP);
          
        }
      }


  /********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        marker.header.stamp =  ros::Time::now();
        marker.ns = ns;

        marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
        marker.pose.orientation.w = 0.0;

        marker.id = id;

        marker.type = visualization_msgs::Marker::POINTS;

        // POINTS markers use x and y scale for width/height respectively
        marker.scale.x = costmap_->getResolution();
        marker.scale.y = costmap_->getResolution();


        // Points are green
        marker.color.g = g;
        marker.color.r = r;
        marker.color.b = b;
        marker.color.a = 1.0;

    }

void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
        marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
        marker.header.stamp =  ros::Time::now();
        marker.ns = ns;

        marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
        marker.pose.orientation.w = 0.0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;

        marker.id = id;

        marker.type = visualization_msgs::Marker::SPHERE;

        //Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
        marker.scale.x = marker.scale.y = 0.5;
       // marker.scale.y = costmap_->getResolution();

        // Points are green
        marker.color.g = g;
        marker.color.r = r;
        marker.color.b = b;
        marker.color.a = 1.0;

    }

void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
        //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0; //¿?

        marker.points.push_back(p); //anyado el punto inicial
        where.publish(marker); //lo publico
        //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
        }

void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
        //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        where.publish(marker); //lo publico
        //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
        }

void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
        unsigned int mpose_x, mpose_y;
        double wpose_x, wpose_y;
        costmap_->indexToCells(index, mpose_x, mpose_y);
        costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
        visualizaCoords(where, marker, wpose_x, wpose_y);

    }

 void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
     for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i)
       {
         unsigned int mpose_x, mpose_y;
         double wpose_x, wpose_y;
         costmap_->indexToCells(*i, mpose_x, mpose_y);
         costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
         //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
         geometry_msgs::Point p;
         p.x = wpose_x;
         p.y = wpose_y;
         p.z = 0; //¿?

         marker.points.push_back(p);
        }
      where.publish(marker);
 //quitar neighborCells de points .popback
 }
 void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
    if (!marker.points.empty()){
        marker.action = visualization_msgs::Marker::DELETE;
        where.publish(marker);
        marker.action = visualization_msgs::Marker::ADD;
    }
    marker.points.clear();


 }
}
