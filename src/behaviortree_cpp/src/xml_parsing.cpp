/*  Copyright (C) 2018-2020 Davide Faconti, Eurecat -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <functional>
#include <list>

#if defined(__linux) || defined(__linux__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#endif

#ifdef _MSC_VER
#pragma warning(disable : 4996)   // do not complain about sprintf
#endif

#include <map>
#include "behaviortree_cpp/xml_parsing.h"
#include "tinyxml2/tinyxml2.h"
#include <filesystem>

#ifdef USING_ROS
#include <ros/package.h>
#endif

#ifdef USING_ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/demangle_util.h"

#include <ros/ros.h>

namespace BT
{
using namespace tinyxml2;

auto StrEqual = [](const char* str1, const char* str2) -> bool {
  return strcmp(str1, str2) == 0;
};



struct SubtreeModel
{
  std::unordered_map<std::string, BT::PortInfo> ports;
};

struct XMLParser::PImpl
{
  TreeNode::Ptr createNodeFromXML(const XMLElement* element,
                                  const Blackboard::Ptr& blackboard,
                                  const TreeNode::Ptr& node_parent,
                                  const std::string &prefix_path,
                                  Tree& output_tree);

  void recursivelyCreateSubtree(const std::string& tree_ID,
                                const std::string &tree_path,
                                const std::string &prefix_path,
                                Tree& output_tree,
                                Blackboard::Ptr blackboard,
                                const TreeNode::Ptr& root_node);

  void getPortsRecursively(const XMLElement* element,
                           std::vector<std::string>& output_ports);

  void loadDocImpl(tinyxml2::XMLDocument* doc, bool add_includes);

  std::list<std::unique_ptr<tinyxml2::XMLDocument> > opened_documents;
  std::map<std::string, const XMLElement*> tree_roots;

  const BehaviorTreeFactory& factory;

  std::filesystem::path current_path;
  std::map<std::string, SubtreeModel> subtree_models;

  int suffix_count;
  
  explicit PImpl(const BehaviorTreeFactory& fact) :
    factory(fact), current_path(std::filesystem::current_path()), suffix_count(0)
  {}

  void clear()
  {
    suffix_count = 0;
    current_path = std::filesystem::current_path();
    opened_documents.clear();
    tree_roots.clear();
  }

private:
  void loadSubtreeModel(const XMLElement* xml_root);
};

#if defined(__linux) || defined(__linux__)
#pragma GCC diagnostic pop
#endif

XMLParser::XMLParser(const BehaviorTreeFactory& factory) : _p(new PImpl(factory))
{}

XMLParser::XMLParser(XMLParser &&other) noexcept
{
  this->_p = std::move(other._p);
}

XMLParser &XMLParser::operator=(XMLParser &&other) noexcept
{
  this->_p = std::move(other._p);
  return *this;
}

XMLParser::~XMLParser()
{}

void XMLParser::loadFromFile(const std::filesystem::path& filepath, bool add_includes)
{
  _p->opened_documents.emplace_back(new tinyxml2::XMLDocument());

  tinyxml2::XMLDocument* doc = _p->opened_documents.back().get();
  doc->LoadFile(filepath.string().c_str());

  _p->current_path = std::filesystem::absolute(filepath.parent_path());

  _p->loadDocImpl(doc, add_includes);
}

void XMLParser::loadFromText(const std::string& xml_text, bool add_includes)
{
  _p->opened_documents.emplace_back(new tinyxml2::XMLDocument());

  tinyxml2::XMLDocument* doc = _p->opened_documents.back().get();
  doc->Parse(xml_text.c_str(), xml_text.size());

  _p->loadDocImpl(doc, add_includes);
}

std::vector<std::string> XMLParser::registeredBehaviorTrees() const
{
  std::vector<std::string> out;
  for (const auto& it : _p->tree_roots)
  {
    out.push_back(it.first);
  }
  return out;
}

void BT::XMLParser::PImpl::loadSubtreeModel(const XMLElement* xml_root)
{
  for (auto models_node = xml_root->FirstChildElement("TreeNodesModel");
       models_node != nullptr;
       models_node = models_node->NextSiblingElement("TreeNodesModel"))
  {
    for (auto sub_node = models_node->FirstChildElement("SubTree");
         sub_node != nullptr;
         sub_node = sub_node->NextSiblingElement("SubTree"))
    {
      auto subtree_id = sub_node->Attribute("ID");
      auto& subtree_model = subtree_models[subtree_id];

      std::pair<const char*, BT::PortDirection>  port_types[3]=
          { {"input_port", BT::PortDirection::INPUT},
           {"output_port", BT::PortDirection::OUTPUT},
           {"inout_port", BT::PortDirection::INOUT} };

      for(const auto& [name, direction]: port_types)
      {
        for (auto port_node = sub_node->FirstChildElement(name);
             port_node != nullptr;
             port_node = port_node->NextSiblingElement(name))
        {
          BT::PortInfo port(direction);
          auto name =  port_node->Attribute("name");
          if(!name)
          {
            ROS_ERROR("Missing attribute [name] in port (SubTree model)");
            throw RuntimeError("Missing attribute [name] in port (SubTree model)");
          }
          if(auto default_value = port_node->Attribute("default"))
          {
            port.setDefaultValue(default_value);
          }
          if(auto description = port_node->Attribute("description"))
          {
            port.setDescription(description);
          }
          subtree_model.ports[name] = std::move(port);
        }
      }
    }
  }
}

void XMLParser::PImpl::loadDocImpl(tinyxml2::XMLDocument* doc, bool add_includes)
{
  if (doc->Error())
  {
    char buffer[200];
    ROS_ERROR("%s", doc->ErrorStr());
    sprintf(buffer, "Error parsing the XML: %s", doc->ErrorStr());
    
    throw RuntimeError(buffer);
  }

  const XMLElement* xml_root = doc->RootElement();

  auto format = xml_root->Attribute("BTCPP_format");
  if(!format)
  {
    std::cout << "Warnings: The first tag of the XML (<root>) should contain the attribute [BTCPP_format=\"4\"]\n"
              << "Please check if your XML is compatible with version 4.x of BT.CPP" << std::endl;
  }

  // recursively include other files
  for (auto incl_node = xml_root->FirstChildElement("include"); incl_node != nullptr;
       incl_node = incl_node->NextSiblingElement("include"))
  {
    if (!add_includes)
    {
      break;
    }

    std::filesystem::path file_path(incl_node->Attribute("path"));
    const char* ros_pkg_relative_path = incl_node->Attribute("ros_pkg");

    if (ros_pkg_relative_path)
    {
      if (file_path.is_absolute())
      {
        ROS_WARN("WARNING: <include path=\"...\"> contains an absolute path.\nAttribute [ros_pkg] will be ignored.");
        std::cout << "WARNING: <include path=\"...\"> contains an absolute path.\n"
                  << "Attribute [ros_pkg] will be ignored." << std::endl;
      }
      else
      {
        std::string ros_pkg_path;
#ifdef USING_ROS
        ros_pkg_path = ros::package::getPath(ros_pkg_relative_path);
#elif defined USING_ROS2
        ros_pkg_path =
            ament_index_cpp::get_package_share_directory(ros_pkg_relative_path);
#else
        throw RuntimeError("Using attribute [ros_pkg] in <include>, but this library was "
                           "compiled without ROS support. Recompile the BehaviorTree.CPP "
                           "using catkin");
#endif
        file_path = std::filesystem::path(ros_pkg_path) / file_path;
      }
    }

    if (!file_path.is_absolute())
    {
      file_path = current_path / file_path;
    }

    opened_documents.emplace_back(new tinyxml2::XMLDocument());
    tinyxml2::XMLDocument* next_doc = opened_documents.back().get();

    // change current path to the included file for handling additional relative paths
    const auto previous_path = current_path;
    current_path = std::filesystem::absolute(file_path.parent_path());

    next_doc->LoadFile(file_path.string().c_str());
    loadDocImpl(next_doc, add_includes);

    // reset current path to the previous value
    current_path = previous_path;
  }

  // Collect the names of all nodes registered with the behavior tree factory
  std::unordered_map<std::string, BT::NodeType> registered_nodes;
  for (const auto& it : factory.manifests())
  {
    registered_nodes.insert({it.first, it.second.type});
  }

  XMLPrinter printer;
  doc->Print(&printer);
  auto xml_text = std::string(printer.CStr(), size_t(printer.CStrSize()));

  // Verify the validity of the XML before adding any behavior trees to the parser's list of registered trees
  VerifyXML(xml_text, registered_nodes);

  loadSubtreeModel(xml_root);

  // Register each BehaviorTree within the XML
  for (auto bt_node = xml_root->FirstChildElement("BehaviorTree"); bt_node != nullptr;
       bt_node = bt_node->NextSiblingElement("BehaviorTree"))
  {
    std::string tree_name;
    if (bt_node->Attribute("ID"))
    {
      tree_name = bt_node->Attribute("ID");
    }
    else
    {
      tree_name = "BehaviorTree_" + std::to_string(suffix_count++);
    }

    tree_roots[tree_name] = bt_node;
  }
}

void VerifyXML(const std::string& xml_text,
               const std::unordered_map<std::string, BT::NodeType>& registered_nodes)
{
  tinyxml2::XMLDocument doc;
  auto xml_error = doc.Parse(xml_text.c_str(), xml_text.size());
  if (xml_error)
  {
    char buffer[512];
    ROS_ERROR("%s Error parsing the XML: %s", buffer, doc.ErrorName());
    sprintf(buffer, "Error parsing the XML: %s", doc.ErrorName());
    throw RuntimeError(buffer);
  }

  //-------- Helper functions (lambdas) -----------------
  auto ThrowError = [&](int line_num, const std::string& text) {
    char buffer[512];
    ROS_ERROR("%s Error at line %d: -> %s", buffer, line_num, text.c_str());
    sprintf(buffer, "Error at line %d: -> %s", line_num, text.c_str());
    throw RuntimeError(buffer);
  };

  auto ChildrenCount = [](const XMLElement* parent_node) {
    int count = 0;
    for (auto node = parent_node->FirstChildElement(); node != nullptr;
         node = node->NextSiblingElement())
    {
      count++;
    }
    return count;
  };
  //-----------------------------

  const XMLElement* xml_root = doc.RootElement();

  if (!xml_root || !StrEqual(xml_root->Name(), "root"))
  {
    ROS_ERROR("The XML must have a root node called <root>");
    throw RuntimeError("The XML must have a root node called <root>");
  }
  //-------------------------------------------------
  auto models_root = xml_root->FirstChildElement("TreeNodesModel");
  auto meta_sibling =
      models_root ? models_root->NextSiblingElement("TreeNodesModel") : nullptr;

  if (meta_sibling)
  {
    ROS_ERROR("%d Only a single node <TreeNodesModel> is supported", meta_sibling->GetLineNum());
    ThrowError(meta_sibling->GetLineNum(), " Only a single node <TreeNodesModel> is "
                                           "supported");
  }
  if (models_root)
  {
    // not having a MetaModel is not an error. But consider that the
    // Graphical editor needs it.
    for (auto node = xml_root->FirstChildElement(); node != nullptr;
         node = node->NextSiblingElement())
    {
      const std::string name = node->Name();
      if (name == "Action" || name == "Decorator" ||
          name == "SubTree" || name == "Condition" ||
          name == "Control")
      {
        const char* ID = node->Attribute("ID");
        if (!ID)
        {
          ROS_ERROR("Error at line %d: -> The attribute, [ID] is mandatory", node->GetLineNum());
          ThrowError(node->GetLineNum(), "Error at line %d: -> The attribute "
                                         "[ID] is mandatory");
        }
      }
    }
  }
  //-------------------------------------------------

  // function to be called recursively
  std::function<void(const XMLElement*)> recursiveStep;

  recursiveStep = [&](const XMLElement* node) {
    const int children_count = ChildrenCount(node);
    const std::string name = node->Name();
    if (name == "Decorator")
    {
      if (children_count != 1)
      {
        ROS_ERROR("%d The node <Decorator> must have exactly 1 child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Decorator> must have exactly 1 "
                                       "child");
      }
      if (!node->Attribute("ID"))
      {
        ROS_ERROR("%d The node <Decorator> must have the attribute [ID]", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Decorator> must have the "
                                       "attribute [ID]");
      }
    }
    else if (name == "Action")
    {
      if (children_count != 0)
      {
        ROS_ERROR("%d The node <Action> must not have any child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Action> must not have any "
                                       "child");
      }
      if (!node->Attribute("ID"))
      {
        ROS_ERROR("%d The node <Action> must have the attribute [ID]", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Action> must have the "
                                       "attribute [ID]");
      }
    }
    else if (name == "Condition")
    {
      if (children_count != 0)
      {
        ROS_ERROR("%d The node <Condition> must not have any child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Condition> must not have any "
                                       "child");
      }
      if (!node->Attribute("ID"))
      {
        ROS_ERROR("%d The node <Condition> must have the attribute [ID]", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Condition> must have the "
                                       "attribute [ID]");
      }
    }
    else if (name == "Control")
    {
      if (children_count == 0)
      {
        ROS_ERROR("%d The node <Control> must have at least 1 child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Control> must have at least 1 "
                                       "child");
      }
      if (!node->Attribute("ID"))
      {
        ROS_ERROR("%d The node <Control> must have the attribute [ID]", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <Control> must have the "
                                       "attribute [ID]");
      }
    }
    else if (name == "Sequence" || name == "ReactiveSequence" ||
        name == "SequenceWithMemory" || name == "Fallback")
    {
      if (children_count == 0)
      {
        ROS_ERROR("%d A Control node must have at least 1 child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "A Control node must have at least 1 "
                                       "child");
      }
    }
    else if (name == "SubTree")
    {
      auto child = node->FirstChildElement();

      if (child)
      {
        if (StrEqual(child->Name(), "remap"))
        {
        ROS_ERROR("%d <remap> was deprecated", node->GetLineNum());
          ThrowError(node->GetLineNum(), "<remap> was deprecated");
        }
        else
        {
        ROS_ERROR("%d <SubTree> should not have any child", node->GetLineNum());
          ThrowError(node->GetLineNum(), "<SubTree> should not have any child");
        }
      }

      if (!node->Attribute("ID"))
      {
        ROS_ERROR("%d The node <SubTree> must have the attribute [ID]", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <SubTree> must have the "
                                       "attribute [ID]");
      }
    }
    else if (name == "BehaviorTree")
    {
      if (children_count != 1)
      {
        ROS_ERROR("%d The node <BehaviorTree> must have exactly 1 child", node->GetLineNum());
        ThrowError(node->GetLineNum(), "The node <BehaviorTree> must have exactly 1 "
                                       "child");
      }
    }
    else
    {
      // search in the factory and the list of subtrees
      const auto search = registered_nodes.find(name);
      bool found = (search != registered_nodes.end());
      if (!found)
      {
        ROS_ERROR("%d Node not recognized: %s", node->GetLineNum(), name.c_str());
        ThrowError(node->GetLineNum(), std::string("Node not recognized: ") + name);
      }

      if (search->second == NodeType::DECORATOR)
      {
        if (children_count != 1)
        {
          ROS_ERROR("%d The node < %s > must have exactly 1 child", node->GetLineNum(), name.c_str());
          ThrowError(node->GetLineNum(),
                     std::string("The node <") + name + "> must have exactly 1 child");
        }
      }
    }
    //recursion
    if (name != "SubTree" )
    {
      for (auto child = node->FirstChildElement(); child != nullptr;
           child = child->NextSiblingElement())
      {
        recursiveStep(child);
      }
    }
  };

  for (auto bt_root = xml_root->FirstChildElement("BehaviorTree"); bt_root != nullptr;
       bt_root = bt_root->NextSiblingElement("BehaviorTree"))
  {
    recursiveStep(bt_root);
  }
}

Tree XMLParser::instantiateTree(const Blackboard::Ptr& root_blackboard,
                                std::string main_tree_ID)
{
  Tree output_tree;

  // use the main_tree_to_execute argument if it was provided by the user
  // or the one in the FIRST document opened
  if (main_tree_ID.empty())
  {
    XMLElement* first_xml_root = _p->opened_documents.front()->RootElement();

    if (auto main_tree_attribute = first_xml_root->Attribute("main_tree_to_execute"))
    {
      main_tree_ID = main_tree_attribute;
    }
    else if (_p->tree_roots.size() == 1)
    {
      // special case: there is only one registered BT.
      main_tree_ID = _p->tree_roots.begin()->first;
    }
    else
    {
      ROS_ERROR("[main_tree_to_execute] was not specified correctly");
      throw RuntimeError("[main_tree_to_execute] was not specified correctly");
    }
  }

  //--------------------------------------
  if (!root_blackboard)
  {
    ROS_ERROR("XMLParser::instantiateTree needs a non-empty root_blackboard");
    throw RuntimeError("XMLParser::instantiateTree needs a non-empty "
                       "root_blackboard");
  }

  _p->recursivelyCreateSubtree(main_tree_ID, {}, {},
                               output_tree, root_blackboard, TreeNode::Ptr());
  output_tree.initialize();
  return output_tree;
}

void XMLParser::clearInternalState()
{
  _p->clear();
}

TreeNode::Ptr XMLParser::PImpl::createNodeFromXML(const XMLElement* element,
                                                  const Blackboard::Ptr& blackboard,
                                                  const TreeNode::Ptr& node_parent,
                                                  const std::string& prefix_path,
                                                  Tree& output_tree)
{
  const auto element_name = element->Name();
  const auto element_ID = element->Attribute("ID");

  auto node_type = convertFromString<NodeType>(element_name);
  // name used by the factory
  std::string type_ID;

  if (node_type == NodeType::UNDEFINED)
  {
    // This is the case of nodes like <MyCustomAction>
    // check if the factory has this name
    if (factory.builders().count(element_name) == 0)
    {
      ROS_ERROR("%s is not a registered node", std::string(element_name).c_str());
      throw RuntimeError(element_name, " is not a registered node");
    }
    type_ID = element_name;

    if (element_ID)
    {
      ROS_ERROR("Attribute [ID] is not allowed in < %s >", type_ID.c_str());
      throw RuntimeError("Attribute [ID] is not allowed in <", type_ID, ">");
    }
  }
  else
  {
    // in this case, it is mandatory to have a field "ID"
    if (!element_ID)
    {
      ROS_ERROR("Attribute [ID] is mandatory in <%s >", type_ID.c_str());
      throw RuntimeError("Attribute [ID] is mandatory in <", type_ID, ">");
    }
    type_ID = element_ID;
  }

  // By default, the instance name is equal to ID, unless the
  // attribute [name] is present.
  const char* attr_name = element->Attribute("name");
  const std::string instance_name = (attr_name != nullptr) ? attr_name : type_ID;

  const TreeNodeManifest* manifest = nullptr;

  auto manifest_it =  factory.manifests().find(type_ID);
  if(manifest_it != factory.manifests().end())
  {
    manifest = &manifest_it->second;
  }

  PortsRemapping port_remap;
  for (const XMLAttribute* att = element->FirstAttribute(); att; att = att->Next())
  {
    if (IsAllowedPortName(att->Name()))
    {
      const std::string attribute_name = att->Name();
      port_remap[attribute_name] = att->Value();
    }
  }

  NodeConfig config;
  config.blackboard = blackboard;
  config.path = prefix_path + instance_name;
  config.uid = output_tree.getUID();
  config.manifest = manifest;

  if(type_ID == instance_name)
  {
    config.path += std::string("::") + std::to_string(config.uid);
  }

  auto AddCondition = [&](auto& conditions, const char* attr_name, auto ID) {
    if (auto script = element->Attribute(attr_name))
    {
      conditions.insert({ID, std::string(script)});
    }
  };

  for(int i=0; i<int(PreCond::COUNT_); i++)
  {
    auto pre = static_cast<PreCond>(i);
    AddCondition(config.pre_conditions, toStr(pre).c_str(), pre);
  }
  for(int i=0; i<int(PostCond::COUNT_); i++)
  {
    auto post = static_cast<PostCond>(i);
    AddCondition(config.post_conditions, toStr(post).c_str(), post);
  }

  //---------------------------------------------
  TreeNode::Ptr new_node;

  if (node_type == NodeType::SUBTREE)
  {
    config.input_ports = port_remap;
    new_node = factory.instantiateTreeNode(instance_name,
                                           toStr(NodeType::SUBTREE),
                                           config);
    auto subtree_node = dynamic_cast<SubTreeNode*>(new_node.get());
    subtree_node->setSubtreeID(type_ID);
  }
  else
  {
    if(!manifest)
    {
      auto msg = StrCat("Missing manifest for element_ID: ", element_ID, ". It shouldn't happen. Please report this issue.");
      ROS_ERROR("%s", msg.c_str());
      throw RuntimeError(msg);
    }

    //Check that name in remapping can be found in the manifest
    for (const auto& [name_in_subtree, _] : port_remap)
    {
      if (manifest->ports.count(name_in_subtree) == 0)
      {
        ROS_ERROR("Possible typo? In the XML, you tried to remap port %s in node [%s(type:%s)], but the manifest of this node does not contain a port with this name", name_in_subtree.c_str(), config.path.c_str(), type_ID.c_str());
        throw RuntimeError("Possible typo? In the XML, you tried to remap port \"",
                           name_in_subtree, "\" in node [", config.path, "(type ", type_ID,
                           ")], but the manifest of this node does not contain a port "
                           "with this name.");
      }
    }

    // Initialize the ports in the BB to set the type
    for (const auto& [port_name, port_info] : manifest->ports)
    {
      auto remap_it = port_remap.find(port_name);
      if (remap_it == port_remap.end())
      {
        continue;
      }
      StringView remapped_port = remap_it->second;

      if (auto param_res = TreeNode::getRemappedKey(port_name, remapped_port))
      {
        // port_key will contain the key to find the entry in the blackboard
        const auto port_key = static_cast<std::string>(param_res.value());

        // if the entry already exists, check that the type is the same
        if (auto prev_info = blackboard->entryInfo(port_key))
        {
          // Check consistency of types.
          bool const port_type_mismatch = (prev_info->isStronglyTyped() &&
                                           port_info.isStronglyTyped() &&
                                           prev_info->type() != port_info.type());

          // special case related to convertFromString
          bool const string_input = (prev_info->type() == typeid(std::string));

          if(port_type_mismatch && !string_input)
          {
            blackboard->debugMessage();

            ROS_ERROR("The creation of the tree failed because the port [",
                               port_key, "] was initially created with type [",
                               demangle(prev_info->type()), "] and, later type [",
                               demangle(port_info.type()), "] was used somewhere else.");

            throw RuntimeError("The creation of the tree failed because the port [",
                               port_key, "] was initially created with type [",
                               demangle(prev_info->type()), "] and, later type [",
                               demangle(port_info.type()), "] was used somewhere else.");
          }
        }
        else
        {
          // not found, insert for the first time.
          blackboard->createEntry(port_key, port_info);
        }
      }
    }

    // Set the port direction in config
    for (const auto& remap_it : port_remap)
    {
      const auto& port_name = remap_it.first;
      auto port_it = manifest->ports.find(port_name);
      if (port_it != manifest->ports.end())
      {
        auto direction = port_it->second.direction();
        if (direction != PortDirection::OUTPUT)
        {
          config.input_ports.insert(remap_it);
        }
        if (direction != PortDirection::INPUT)
        {
          config.output_ports.insert(remap_it);
        }
      }
    }

    // use default value if available for empty ports. Only inputs
    for (const auto& port_it : manifest->ports)
    {
      const std::string& port_name = port_it.first;
      const PortInfo& port_info = port_it.second;

      auto direction = port_info.direction();

      if (direction != PortDirection::OUTPUT &&
          config.input_ports.count(port_name) == 0 &&
          !port_info.defaultValue().empty())
      {
        try {
          config.input_ports.insert({port_name, port_info.defaultValueString()});
        }
        catch(LogicError&) {}
      }
    }

    new_node = factory.instantiateTreeNode(instance_name, type_ID, config);
  }

  // add the pointer of this node to the parent
  if (node_parent != nullptr)
  {
    if (auto control_parent = dynamic_cast<ControlNode*>(node_parent.get()))
    {
      control_parent->addChild(new_node.get());
    }
    else if (auto decorator_parent = dynamic_cast<DecoratorNode*>(node_parent.get()))
    {
      decorator_parent->setChild(new_node.get());
    }
  }

  return new_node;
}

void BT::XMLParser::PImpl::recursivelyCreateSubtree(
    const std::string& tree_ID,
    const std::string& tree_path,
    const std::string& prefix_path,
    Tree& output_tree,
    Blackboard::Ptr blackboard,
    const TreeNode::Ptr& root_node)
{
  std::function<void(const TreeNode::Ptr&, Tree::Subtree::Ptr, std::string, const XMLElement*)>
      recursiveStep;

  recursiveStep = [&](TreeNode::Ptr parent_node,
                      Tree::Subtree::Ptr subtree,
                      std::string prefix,
                      const XMLElement* element)
  {
    // create the node
    auto node = createNodeFromXML(element, blackboard, parent_node, prefix, output_tree);
    subtree->nodes.push_back(node);

    // common case: iterate through all children
    if (node->type() != NodeType::SUBTREE)
    {
      for (auto child_element = element->FirstChildElement(); child_element;
           child_element = child_element->NextSiblingElement())
      {
        recursiveStep(node, subtree, prefix, child_element);
      }
    }
    else   // special case: SubTreeNode
    {
      auto new_bb = Blackboard::create(blackboard);
      const std::string subtree_ID = element->Attribute("ID");
      std::unordered_map<std::string, std::string> remapping;
      bool do_autoremap = false;

      for (auto attr = element->FirstAttribute(); attr != nullptr; attr = attr->Next())
      {
        const char* attr_name = attr->Name();
        const char* attr_value = attr->Value();

        if (StrEqual(attr_name, "_autoremap"))
        {
          do_autoremap = convertFromString<bool>(attr_value);
          new_bb->enableAutoRemapping(do_autoremap);
          continue;
        }
        if (!IsAllowedPortName(attr->Name()))
        {
          continue;
        }
        remapping.insert( {attr_name, attr_value} );
      }
      // check if this subtree has a model. If it does,
      // we want o check if all the mandatory ports were remapped and
      // add default ones, if necessary
      auto subtree_model_it = subtree_models.find(subtree_ID);
      if(subtree_model_it != subtree_models.end())
      {
        const auto& subtree_model_ports = subtree_model_it->second.ports;
        // check if:
        // - remapping contains mondatory ports
        // - if any of these has default value
        for(const auto& [port_name, port_info]: subtree_model_ports)
        {
          auto it = remapping.find(port_name);
          // don't override existing remapping
          if( it == remapping.end() && !do_autoremap)
          {
            // remapping is not explicitly defined in the XML: use the model
            if(port_info.defaultValueString().empty())
            {
              auto msg = StrCat("In the <TreeNodesModel> the <Subtree ID=\"", subtree_ID,
                                "\"> is defining a mandatory port called [",
                                port_name,"], but you are not remapping it");
              ROS_ERROR(msg.c_str());
              throw RuntimeError(msg);
            }
            else {
              remapping.insert({port_name, port_info.defaultValueString()});
            }
          }
        }
      }

      for(const auto& [attr_name, attr_value]: remapping)
      {
        if (TreeNode::isBlackboardPointer(attr_value))
        {
          // do remapping
          StringView port_name = TreeNode::stripBlackboardPointer(attr_value);
          new_bb->addSubtreeRemapping(attr_name, port_name);
        }
        else
        {
          // constant string: just set that constant value into the BB
          // IMPORTANT: this must not be autoremapped!!!
          new_bb->enableAutoRemapping(false);
          new_bb->set(attr_name, static_cast<std::string>(attr_value));
          new_bb->enableAutoRemapping(do_autoremap);
        }
      }

      std::string subtree_path = subtree->instance_name;
      if(!subtree_path.empty()) {
        subtree_path += "/";
      }
      if(auto name = element->Attribute("name") ) {
        subtree_path +=  name;
      }
      else {
        subtree_path += subtree_ID + "::" + std::to_string(node->UID());
      }

      recursivelyCreateSubtree(subtree_ID,
                               subtree_path, // name
                               subtree_path + "/",  //prefix
                               output_tree, new_bb, node);
    }
  };

  auto it = tree_roots.find(tree_ID);
  if (it == tree_roots.end())
  {
    ROS_ERROR("Can't find a tree with name: %s", tree_ID.c_str());
    throw std::runtime_error(std::string("Can't find a tree with name: ") + tree_ID);
  }

  auto root_element = it->second->FirstChildElement();

  //-------- start recursion -----------

  // Append a new subtree to the list
  auto new_tree = std::make_shared<Tree::Subtree>();
  new_tree->blackboard = blackboard;
  new_tree->instance_name = tree_path;
  new_tree->tree_ID = tree_ID;
  output_tree.subtrees.push_back(new_tree);

  recursiveStep(root_node, new_tree, prefix_path, root_element);
}

void XMLParser::PImpl::getPortsRecursively(const XMLElement* element,
                                           std::vector<std::string>& output_ports)
{
  for (const XMLAttribute* attr = element->FirstAttribute(); attr != nullptr;
       attr = attr->Next())
  {
    const char* attr_name = attr->Name();
    const char* attr_value = attr->Value();
    if (IsAllowedPortName(attr_name) && TreeNode::isBlackboardPointer(attr_value))
    {
      auto port_name = TreeNode::stripBlackboardPointer(attr_value);
      output_ports.push_back(static_cast<std::string>(port_name));
    }
  }

  for (auto child_element = element->FirstChildElement(); child_element;
       child_element = child_element->NextSiblingElement())
  {
    getPortsRecursively(child_element, output_ports);
  }
}

void addNodeModelToXML(const TreeNodeManifest& model,
                          XMLDocument& doc,
                          XMLElement* model_root)
{
  XMLElement* element = doc.NewElement(toStr(model.type).c_str());
  element->SetAttribute("ID", model.registration_ID.c_str());

  for (const auto& [port_name, port_info] : model.ports)
  {
    XMLElement* port_element = nullptr;
    switch (port_info.direction())
    {
      case PortDirection::INPUT:
        port_element = doc.NewElement("input_port");
        break;
      case PortDirection::OUTPUT:
        port_element = doc.NewElement("output_port");
        break;
      case PortDirection::INOUT:
        port_element = doc.NewElement("inout_port");
        break;
    }

    port_element->SetAttribute("name", port_name.c_str());
    if (port_info.type() != typeid(void))
    {
      port_element->SetAttribute("type", BT::demangle(port_info.type()).c_str());
    }
    if (!port_info.defaultValue().empty())
    {
      port_element->SetAttribute("default", port_info.defaultValueString().c_str());
    }

    if (!port_info.description().empty())
    {
      port_element->SetText(port_info.description().c_str());
    }
    element->InsertEndChild(port_element);
  }

  if (!model.description.empty())
  {
    auto description_element = doc.NewElement("description");
    description_element->SetText(model.description.c_str());
    element->InsertEndChild(description_element);
  }

  model_root->InsertEndChild(element);
}

void addTreeToXML(const Tree& tree,
                  XMLDocument& doc,
                  XMLElement* rootXML,
                  bool add_metadata,
                  bool add_builtin_models)
{  
  std::function<void(const TreeNode&, XMLElement*)> addNode;
  addNode = [&](const TreeNode& node,
                XMLElement* parent_elem)
  {
    XMLElement* elem = nullptr;

    if (auto subtree = dynamic_cast<const SubTreeNode*>(&node))
    {
      elem = doc.NewElement(node.registrationName().c_str());
      elem->SetAttribute("ID", subtree->subtreeID().c_str());
      if(add_metadata){
        elem->SetAttribute("_fullpath", subtree->config().path.c_str());
      }
    }
    else {
      elem = doc.NewElement(node.registrationName().c_str());
      elem->SetAttribute("name", node.name().c_str());
    }

    if(add_metadata){
      elem->SetAttribute("_uid", node.UID());
    }

    for(const auto& [name, value]: node.config().input_ports)
    {
      elem->SetAttribute(name.c_str(), value.c_str());
    }
    for(const auto& [name, value]: node.config().output_ports)
    {
      // avoid duplicates, in the case of INOUT ports
      if(node.config().input_ports.count(name) == 0)
      {
        elem->SetAttribute(name.c_str(), value.c_str());
      }
    }

    for(const auto& [pre, script]: node.config().pre_conditions)
    {
      elem->SetAttribute(toStr(pre).c_str(), script.c_str());
    }
    for(const auto& [post, script]: node.config().post_conditions)
    {
      elem->SetAttribute(toStr(post).c_str(), script.c_str());
    }

    parent_elem->InsertEndChild(elem);

    if (auto control = dynamic_cast<const ControlNode*>(&node))
    {
      for (const auto& child : control->children())
      {
        addNode(*child, elem);
      }
    }
    else if (auto decorator = dynamic_cast<const DecoratorNode*>(&node))
    {
      if(decorator->type() != NodeType::SUBTREE)
      {
        addNode(*decorator->child(), elem);
      }
    }
  };

  for(const auto& subtree: tree.subtrees)
  {
    XMLElement* subtree_elem = doc.NewElement("BehaviorTree");
    subtree_elem->SetAttribute("ID", subtree->tree_ID.c_str());
    subtree_elem->SetAttribute("_fullpath", subtree->instance_name.c_str());
    rootXML->InsertEndChild(subtree_elem);
    addNode(*subtree->nodes.front(), subtree_elem);
  }

  XMLElement* model_root = doc.NewElement("TreeNodesModel");
  rootXML->InsertEndChild(model_root);

  static const BehaviorTreeFactory temp_factory;

  std::map<std::string, const TreeNodeManifest*> ordered_models;
  for (const auto& [registration_ID, model] : tree.manifests)
  {
    if(add_builtin_models || !temp_factory.builtinNodes().count(registration_ID))
    {
      ordered_models.insert( {registration_ID, &model} );
    }
  }

  for (const auto& [registration_ID, model] : ordered_models)
  {
    addNodeModelToXML(*model, doc, model_root);
  }
}

std::string writeTreeNodesModelXML(const BehaviorTreeFactory& factory,
                                   bool include_builtin)
{
  XMLDocument doc;

  XMLElement* rootXML = doc.NewElement("root");
  rootXML->SetAttribute("BTCPP_format", "4");
  doc.InsertFirstChild(rootXML);

  XMLElement* model_root = doc.NewElement("TreeNodesModel");
  rootXML->InsertEndChild(model_root);

  std::map<std::string, const TreeNodeManifest*> ordered_models;

  for (const auto& [registration_ID, model] : factory.manifests())
  {
    if (include_builtin || factory.builtinNodes().count(registration_ID) == 0)
    {
      ordered_models.insert( {registration_ID, &model} );
    }
  }

  for (const auto& [registration_ID, model] : ordered_models)
  {
    addNodeModelToXML(*model, doc, model_root);
  }

  XMLPrinter printer;
  doc.Print(&printer);
  return std::string(printer.CStr(), size_t(printer.CStrSize() - 1));
}

Tree buildTreeFromText(const BehaviorTreeFactory& factory, const std::string& text,
                       const Blackboard::Ptr& blackboard)
{
  XMLParser parser(factory);
  parser.loadFromText(text);
  return parser.instantiateTree(blackboard);
}

Tree buildTreeFromFile(const BehaviorTreeFactory& factory,
                       const std::string& filename,
                       const Blackboard::Ptr& blackboard)
{
  XMLParser parser(factory);
  parser.loadFromFile(filename);
  return parser.instantiateTree(blackboard);
}

std::string WriteTreeToXML(const Tree &tree, bool add_metadata, bool add_builtin_models)
{
  XMLDocument doc;

  XMLElement* rootXML = doc.NewElement("root");
  rootXML->SetAttribute("BTCPP_format", 4);
  doc.InsertFirstChild(rootXML);

  addTreeToXML(tree, doc, rootXML, add_metadata, add_builtin_models);

  XMLPrinter printer;
  doc.Print(&printer);
  return std::string(printer.CStr(), size_t(printer.CStrSize() - 1));
}

}   // namespace BT
