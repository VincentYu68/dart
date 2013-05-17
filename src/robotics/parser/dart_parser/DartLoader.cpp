/**
 * @file DartLoader.cpp
 */

#include "DartLoader.h"
#include <map>
#include "../urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "simulation/World.h"
#include "kinematics/Joint.h"
#include "kinematics/Transformation.h"
#include "kinematics/Dof.h"

/**
 * @function DartLoader
 * @brief Constructor
 */
DartLoader::DartLoader() {
}

/**
 * @function ~DartLoader
 * @brief Destructor
 */
DartLoader::~DartLoader() {

}

/**
 * @function parseSkeleton
 */
dynamics::SkeletonDynamics* DartLoader::parseSkeleton( std::string _urdfFile,
						       std::string _rootToSkelPath ) {

  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );

  boost::shared_ptr<urdf::ModelInterface> skeletonModel = urdf::parseURDF( xml_string );
  
  return modelInterfaceToSkeleton( skeletonModel, _rootToSkelPath );

}

/**
 * @function parseRobot
 */
dynamics::SkeletonDynamics* DartLoader::parseRobot( std::string _urdfFile,
					 std::string _rootToRobotPath ) {

  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );
  boost::shared_ptr<urdf::ModelInterface> robotModel = urdf::parseURDF( xml_string );
  return modelInterfaceToRobot( robotModel, _rootToRobotPath );
}

/**
 * @function parseObject
 */
dynamics::SkeletonDynamics* DartLoader::parseObject( std::string _urdfFile,
					   std::string _rootToObjectPath) {
  
  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );
  boost::shared_ptr<urdf::ModelInterface> objectModel = urdf::parseURDF( xml_string );
  return modelInterfaceToObject( objectModel, _rootToObjectPath );

}

/**
 * @function parseWorld
 */
simulation::World* DartLoader::parseWorld( std::string _urdfFile ) {

  std::string raw_World_Path = _urdfFile;
 
  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::replace( raw_World_Path.begin(), raw_World_Path.end(), '\\' , '/' );
  mRoot_To_World_Path = raw_World_Path.substr( 0, raw_World_Path.rfind("/") + 1 );

  if(debug) { 
    std::cout<< "[parseWorld] mRoot_To_World_Path :" << mRoot_To_World_Path << std::endl;
  }
  
  simulation::World* world;
  dynamics::SkeletonDynamics* robot;
  dynamics::SkeletonDynamics* object;
  
  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );

  // Store paths from world to entities
  parseWorldToEntityPaths( xml_string );

  boost::shared_ptr<urdf::World> worldInterface =  urdf::parseWorldURDF( xml_string, 
									 mRoot_To_World_Path );

  // If an empty worldInterface pointer returns
  if( !worldInterface ) {
    std::cout<< "[parseWorldURDF] Null world pointer. No loading and exiting!"<<std::endl;
    world = NULL;
    return NULL;
  }
  else {
    world = new simulation::World();

    Eigen::VectorXd pose(6); 

    for( unsigned int i = 0; i < worldInterface->objectModels.size(); ++i ) {

      // Set the corresponding path
      std::string object_path = mRoot_To_World_Path;
      std::string object_localPath = mWorld_To_Entity_Paths.find( worldInterface->objectModels[i].model->getName() )->second;
      object_path.append(object_localPath);

      if( debug ) {
	std::cout<<"Global filepath for: "<<worldInterface->objectModels[i].model->getName() << " is: "<<object_path<<std::endl;
      }     

      object = modelInterfaceToObject(  worldInterface->objectModels[i].model, object_path );

      if( object == NULL ) {
	std::cout<< "[ERROR] Object "<< worldInterface->objectModels[i].model->getName() <<" was not correctly parsed. World is not loaded. Exiting!"<<std::endl;
	world = NULL; 
	return world;
      }

      // Initialize position and RPY 
      pose << 0, 0, 0, 0, 0, 0;
      pose(0) = worldInterface->objectModels[i].origin.position.x;
      pose(1) = worldInterface->objectModels[i].origin.position.y;
      pose(2) = worldInterface->objectModels[i].origin.position.z;
      worldInterface->objectModels[i].origin.rotation.getRPY( pose(3), pose(4), pose(5) );

      kinematics::Joint* joint = object->getRoot()->getParentJoint();
      joint->getTransform(0)->getDof(0)->setValue(pose(0));
      joint->getTransform(0)->getDof(1)->setValue(pose(1));
      joint->getTransform(0)->getDof(2)->setValue(pose(2));
      joint->getTransform(1)->getDof(0)->setValue(pose(5));
      joint->getTransform(2)->getDof(0)->setValue(pose(4));
      joint->getTransform(3)->getDof(0)->setValue(pose(3));
      joint->updateStaticTransform();
      object->initSkel();

      world->addSkeleton( object );
    }
    
    for( unsigned int i = 0; i < worldInterface->robotModels.size(); ++i )  {
      
      // Set the corresponding path
      std::string robot_path = mRoot_To_World_Path;
      std::string robot_localPath = mWorld_To_Entity_Paths.find( worldInterface->robotModels[i].model->getName() )->second;
      robot_path.append(robot_localPath);
      if( debug ) {
	std::cout<<"Global filepath for: "<<worldInterface->robotModels[i].model->getName() << " is: "<<robot_path<<std::endl;
      }
      robot = modelInterfaceToRobot(  worldInterface->robotModels[i].model, robot_path );

      if( robot == NULL ) {
	std::cout<< "[ERROR] Robot "<< worldInterface->robotModels[i].model->getName() <<" was not correctly parsed. World is not loaded. Exiting!"<<std::endl;
	world = NULL; 
	return world;
      }

      // Initialize position and RPY 
      pose << 0, 0, 0, 0, 0, 0;
      pose(0) = worldInterface->robotModels[i].origin.position.x;
      pose(1) = worldInterface->robotModels[i].origin.position.y;
      pose(2) = worldInterface->robotModels[i].origin.position.z;
      worldInterface->robotModels[i].origin.rotation.getRPY( pose(3), pose(4), pose(5) );

      kinematics::Joint* joint = robot->getRoot()->getParentJoint();
      joint->getTransform(0)->getDof(0)->setValue(pose(0));
      joint->getTransform(0)->getDof(1)->setValue(pose(1));
      joint->getTransform(0)->getDof(2)->setValue(pose(2));
      joint->getTransform(1)->getDof(0)->setValue(pose(5));
      joint->getTransform(2)->getDof(0)->setValue(pose(4));
      joint->getTransform(3)->getDof(0)->setValue(pose(3));
      joint->updateStaticTransform();
      robot->initSkel();

      world->addSkeleton( robot );
    }
  } // end else(worldInterface)
  
  return world;
}

/**
 * @function parseWorldToEntityPaths
 */
void DartLoader::parseWorldToEntityPaths( const std::string &_xml_string ) {
    
    TiXmlDocument xml_doc;
    xml_doc.Parse( _xml_string.c_str() );
    
    TiXmlElement *world_xml = xml_doc.FirstChildElement("world");
    
    if( !world_xml ) {
      return;
    }
        
    // Get all include filenames
    std::map<std::string, std::string> includedFiles;

    for( TiXmlElement* include_xml = world_xml->FirstChildElement("include");
	 include_xml; include_xml = include_xml->NextSiblingElement("include") ) {
      
      const char *filename = include_xml->Attribute("filename");
      const char *model_name = include_xml->Attribute("model_name");
      std::string string_filename( filename );
      std::string string_filepath = string_filename.substr( 0, string_filename.rfind("/") + 1 );
      std::string string_model_name( model_name );

      includedFiles[string_model_name] = string_filepath;
    }
    
    // Get all entities
    for( TiXmlElement* entity_xml = world_xml->FirstChildElement("entity");
	 entity_xml; entity_xml = entity_xml->NextSiblingElement("entity") ) {

      // Find model and name for entity, if not, error
	const char* entity_model = entity_xml->Attribute("model");
	const char* entity_name = entity_xml->Attribute("name");
	
	if( entity_name && entity_model ) {
	  std::string string_entity_model( entity_model );  
	  std::string string_entity_name( entity_name ); 
	  // Find the model
	  if( includedFiles.find( string_entity_model ) == includedFiles.end() ) {
	    std::cout<<"[!] Did not find entity model included. Weird things may happen"<<std::endl;
	    return;
	  } 
	  // Add it
	  else {
	    std::string string_entity_filepath = includedFiles.find( string_entity_model )->second;
	    mWorld_To_Entity_Paths[string_entity_name] = string_entity_filepath;	    
	    if(debug) { std::cout<<"Entity name: "<<string_entity_name<<" filepath: "<<string_entity_filepath << std::endl; }
	  }
	}
	// If no name or model is defined
	else {
	  std::cout<< "[!] Entity was not defined. Weird things will happen" <<std::endl;
	}

    } // for all entities

}

/**
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a SkeletonDynamics object
 */
dynamics::SkeletonDynamics* DartLoader::modelInterfaceToSkeleton( boost::shared_ptr<urdf::ModelInterface> _model,
								  std::string _rootToSkelPath ) {
  
  if( _rootToSkelPath.empty() ) {
    std::cout<< "[DartLoader] Absolute path to skeleton "<<_model->getName()<<" is not set. Probably I will crash!"<<std::endl;
  }

  dynamics::SkeletonDynamics* mSkeleton; 
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;
  
  /** Create new skeleton object */
  mSkeleton = new dynamics::SkeletonDynamics();

  /** Set skeleton name */
  mSkeleton->setName( _model->getName() );

  /** Load links and convert them to DART BodyNodes */
  mNodes.resize(0);  
  
  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {
    // If it is world, don't parse (gazebo hack to define rootJoint)
    if( strcmp( (*lk).second->name.c_str(), "world" ) == 0 ) { continue; }

    node = createDartNode( (*lk).second, mSkeleton, _rootToSkelPath );
    if( node == NULL ) { return NULL; }

    mNodes.push_back( node );
  }
  
  if(debug) printf ("** Created %u body nodes \n", mNodes.size() );
  
  /** Initialize Joint store vector */
  mJoints.resize(0);
  
  /** root joint */
  std::string rootName = _model->getRoot()->name;
  rootNode = getNode( rootName );
  
  if(debug) printf ("[DartLoader] Root Node: %s \n", rootNode->getName() );
  
  /** If root node is NULL, nothing to create */
  if( rootNode == NULL ) {
    // Good, we have to set the node attached to world as root
    if( rootName == "world" ) {
      int numRoots = _model->getRoot()->child_links.size();
      if( numRoots != 1 ) { 
	std::cout<< "[ERROR] Not unique link attached to world" <<std::endl; 
      } else {
	rootName = (_model->getRoot()->child_links)[0]->name;
	rootNode = getNode( rootName );
	if( rootNode == NULL ) { return NULL; }
	std::cout<<"[info] World specified in URDF. Root node considered:"<< rootName <<std::endl;
	
	// Since the original rootName was world, add the joint that had it as its parent (only one)
	for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	     jt != _model->joints_.end(); jt++ ) {  
	  if( ( (*jt).second )->parent_link_name == "world" ) {
	    rootJoint = createDartRootJoint( (*jt).second, mSkeleton );
	    if( rootJoint == NULL ) { return NULL; }
	    mJoints.push_back( rootJoint );
	  }
	} // end of else
      } // end of rootName == "world"
    } 
    // Bad. Either the URDF is bad or the structure is not tree-like
    else {
      std::cout << "[ERROR] No root node found!" << std::endl;
      return NULL;
    }
  }
  else {
    /** Create a joint for floating */
    rootJoint =  createNewDartRootJoint( rootNode, mSkeleton ); 
    if( rootJoint == NULL ) { return NULL; }
    mJoints.push_back( rootJoint );
  }   
  
  //-- Save DART structure

  // Push parents first
  std::list<dynamics::BodyNodeDynamics*> nodeStack;
  dynamics::BodyNodeDynamics* u;
  nodeStack.push_back( rootNode );
  
  int numIter = 0;
  while( !nodeStack.empty() && numIter < mNodes.size() ) {
    // Get front element on stack and update it
    u = nodeStack.front();
    // Add it to the Skeleton
    mSkeleton->addNode(u);


    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	 jt != _model->joints_.end(); 
	 jt++ ) {  
      if( ( (*jt).second )->parent_link_name == u->getName() ) {
	joint = createDartJoint( (*jt).second, mSkeleton );
	mJoints.push_back( joint );
      }
    }
    
    // Pop it out
    nodeStack.pop_front();
    
    // Add its kids
    for( int idx = 0; idx < u->getNumChildJoints(); ++idx ) {
      nodeStack.push_back( (dynamics::BodyNodeDynamics*)( u->getChildNode(idx) ) );
    }
    numIter++;
  } // end while
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  if(debug) printf ("[debug] Pushed %d nodes in order \n", numIter );
  
  // Init robot (skeleton)
  mSkeleton->initSkel();
  
  return mSkeleton;
}

/**
 * @function modelInterfaceToRobot
 */
dynamics::SkeletonDynamics* DartLoader::modelInterfaceToRobot( boost::shared_ptr<urdf::ModelInterface> _model,
						    std::string _rootToRobotPath ) {
  

  if( _rootToRobotPath.empty() ) {
    std::cout<< "[DartLoader] Absolute path to robot "<<_model->getName()<<" is not set. Probably I will crash!"<<std::endl;
  }

  dynamics::SkeletonDynamics* mRobot;
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;
  
  /** Create new robot object */
  mRobot = new dynamics::SkeletonDynamics();
  
  /** Set robot name */
  mRobot->setName( _model->getName() );
  
  /** Load links and convert them to DART BodyNodes */
  mNodes.resize(0);  

  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {

    // If it is world, don't parse (gazebo hack to define rootJoint)
    if( strcmp( (*lk).second->name.c_str(), "world" ) == 0 ) { continue; }

    node = createDartNode( (*lk).second, mRobot, _rootToRobotPath );
    if( node == NULL ) { return NULL; }

    mNodes.push_back( node ); 
  }

  if(debug) printf ("** Created %u body nodes \n", mNodes.size() );
  
  /** Initialize Joint store vector */
  mJoints.resize(0);
  
  /** root joint */
  std::string rootName = _model->getRoot()->name;
  rootNode = getNode( rootName );
  
  if(debug) printf ("[DartLoader] Root Node: %s \n", rootNode->getName() );

  /** If root node is NULL, nothing to create */
  if( rootNode == NULL ) {
    // Good, we have to set the node attached to world as root
    if( rootName == "world" ) {
      int numRoots = _model->getRoot()->child_links.size();
      if( numRoots != 1 ) { 
	std::cout<< "[ERROR] Not unique link attached to world" <<std::endl; 
      } else {
	rootName = (_model->getRoot()->child_links)[0]->name;
	rootNode = getNode( rootName );
	if( rootNode == NULL ) { return NULL; }
	std::cout<<"[info] World specified in URDF. Root node considered:"<< rootName <<std::endl;

	// Since the original rootName was world, add the joint that had it as its parent (only one)
	for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	     jt != _model->joints_.end(); jt++ ) {  
	  if( ( (*jt).second )->parent_link_name == "world" ) {
	    rootJoint = createDartRootJoint( (*jt).second, mRobot );
	    if( rootJoint == NULL ) { return NULL; }
	    mJoints.push_back( rootJoint );
	  }
	} // end of for (joint iterator)
      } // end of else ( if there is a unique node attached to world)

    } // end of rootName == "world" (when rootNode == NULL )
    // Bad. Either the URDF is bad or the structure is not tree-like
    else {
      std::cout << "[ERROR] No root node found!" << std::endl;
      return NULL;
    }
  } // end of rootNode == NULL 

  // If rootNode is not NULL and world link is not defined, then we assume the user wants a free floating robot
  else {
    /** Create a joint for floating */
    rootJoint =  createNewDartRootJoint( rootNode, mRobot );
    if( rootJoint == NULL ) { return NULL; } 
    mJoints.push_back( rootJoint );
  }   
  
  //-- Save DART structure

  // Push parents first
  std::list<dynamics::BodyNodeDynamics*> nodeStack;
  dynamics::BodyNodeDynamics* u;
  nodeStack.push_back( rootNode );
  
  int numIter = 0;
  while( !nodeStack.empty() && numIter < mNodes.size() ) {
    // Get front element on stack and update it
    u = nodeStack.front();
    // Add it to the Robot
    mRobot->addNode(u);

    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	 jt != _model->joints_.end(); 
	 jt++ ) {  
      if( ( (*jt).second )->parent_link_name == u->getName() ) {
	joint = createDartJoint( (*jt).second, mRobot );
	mJoints.push_back( joint );
      }
    }
    
    // Pop it out
    nodeStack.pop_front();
    
    // Add its kids
    for( int idx = 0; idx < u->getNumChildJoints(); ++idx ) {
      nodeStack.push_back( (dynamics::BodyNodeDynamics*)( u->getChildNode(idx) ) );
    }
    numIter++;
  } // end while
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  if(debug) printf ("[debug] Pushed %d nodes in order \n", numIter );
  
  // Init robot (skeleton)
  mRobot->initSkel();
  return mRobot;
}

/**
 * @function modelInterfaceToObject
 */
dynamics::SkeletonDynamics* DartLoader::modelInterfaceToObject( boost::shared_ptr<urdf::ModelInterface> _model,
						      std::string _rootToObjectPath ) {
  
  if( _rootToObjectPath.empty() ) {
    std::cout<< "[DartLoader] Absolute path to object "<<_model->getName()<<" is not set. Probably I will crash!"<<std::endl;
    }

  dynamics::SkeletonDynamics* mObject;
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;

  mObject = new dynamics::SkeletonDynamics();
  
  // name
  mObject->setName( _model->getName() );
  
  // BodyNode
  mNodes.resize(0);  
  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {

    // If it is world, don't parse (gazebo hack to define rootJoint)
    if( strcmp( (*lk).second->name.c_str(), "world" ) == 0 ) { continue; }

    node = createDartNode( (*lk).second, mObject, _rootToObjectPath );
    if( node == NULL ) { return NULL; }

    mNodes.push_back( node );
  }
    if(debug) printf ("[debug] Created %u body nodes \n", mNodes.size() );
  
  // Joint
  mJoints.resize(0);
  
  for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
       jt != _model->joints_.end(); 
       jt++ ) {  
    rootJoint = createDartRootJoint( (*jt).second, mObject );
    if( rootJoint == NULL ) { return NULL; }
    mJoints.push_back( joint );

  }
  
  //-- root joint
  rootNode = getNode( _model->getRoot()->name ); // no rootnode
  rootJoint = createNewDartRootJoint( rootNode, mObject );
  if( rootJoint == NULL ) { return NULL; }
  mJoints.push_back( rootJoint );
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  
  //-- Save DART structure

  // 1. Root node was added already so add...
  
  // 2. The rest of nodes
  for( unsigned int i = 0; i < mNodes.size(); ++i ) {
    // Add nodes to the object
      mObject->addNode( mNodes[i] );
  }
  
  // Init object (skeleton)
  mObject->initSkel();
  
  return mObject;
}

/**
 * @function getNode
 */
dynamics::BodyNodeDynamics* DartLoader::getNode( std::string _nodeName ) {

  for( unsigned int i = 0; i < mNodes.size(); ++i ) {
    std::string node( mNodes[i]->getName() );
    if( node ==  _nodeName ) {
      return mNodes[i];
    }
  }
  if(debug) printf ("[getNode] ERROR: Returning  NULL for  %s \n", _nodeName.c_str() );
  return NULL;
}

/**
 * @function readXml
 */
std::string  DartLoader::readXmlToString( std::string _xmlFile ) {
  
  std::string xml_string;
  
  std::fstream xml_file( _xmlFile.c_str(), std::fstream::in );
  
  // Read xml
  while( xml_file.good() ) {
    std::string line;
    std::getline( xml_file, line );
    xml_string += (line + "\n");
  }
  xml_file.close();
  
  return xml_string;
}
