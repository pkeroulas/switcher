/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "quiddity-manager.h"
#include "quiddity.h" 
#include "gst-utils.h"

namespace switcher
{

  QuiddityManager::ptr 
  QuiddityManager::make_manager ()
  {
    QuiddityManager::ptr manager(new QuiddityManager);
    return manager;
  }
  
  QuiddityManager::ptr 
  QuiddityManager::make_manager (std::string name)
  {
    QuiddityManager::ptr manager(new QuiddityManager(name));
    return manager;
  }

  QuiddityManager::QuiddityManager() :
    name_ ("default")
  {
    init_command_sync ();
    manager_impl_ = QuiddityManager_Impl::make_manager ();
  }

  QuiddityManager::QuiddityManager(std::string name) :
    name_ (name)
  {
    init_command_sync ();
    manager_impl_ = QuiddityManager_Impl::make_manager (name);
  }

  QuiddityManager::~QuiddityManager()
  {
    clear_command_sync ();
  }

  std::string
  QuiddityManager::get_name()
  {
    return name_;
  }


  void 
  QuiddityManager::command_lock ()
  {
    g_mutex_lock (seq_mutex_);
    command_.reset (new QuiddityCommand ());
  }

  void 
  QuiddityManager::command_unlock ()
  {
    //command has been invoked with the return value
    //save the command
    command_history_.push_back (command_);
    g_mutex_unlock(seq_mutex_);
  }


  bool
  QuiddityManager::save_command_history (const char *file_path)
  {
    JSONBuilder::ptr builder;
    builder.reset (new JSONBuilder ());
    builder->reset ();
    builder->begin_object ();
    builder->set_member_name ("history");
    builder->begin_array ();
    int i = 0;
    for (auto &it: command_history_)
      builder->add_node_value (it->get_json_root_node ());
    builder->end_array ();
    builder->end_object ();
    
    //FIXME HERE !! actually write the file
    //g_print ("%s",builder->get_string(true).c_str ());
    return true;
  }

  //----------- API -----------------------------
  std::string 
  QuiddityManager::get_properties_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_properties_description,
		       quiddity_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_property_description (std::string quiddity_name, std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property_description,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_properties_description_by_class (std::string class_name)
  {
    return seq_invoke (QuiddityCommand::get_properties_description_by_class,
		       class_name.c_str(),
		       NULL);
  }

  std::string 
  QuiddityManager::get_property_description_by_class (std::string class_name, std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property_description_by_class,
		       class_name.c_str(),
		       property_name.c_str(),
		       NULL);
  }

  bool
  QuiddityManager::set_property (std::string quiddity_name,
				 std::string property_name,
				 std::string property_value)
  {
    std::string res = seq_invoke (QuiddityCommand::set_property,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       property_value.c_str(),
		       NULL);
    if (res == "true")
      return true;
    else 
      return false;
  }

  std::string
  QuiddityManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    return seq_invoke (QuiddityCommand::get_property,
		       quiddity_name.c_str(),
		       property_name.c_str(),
		       NULL);
  }


  bool 
  QuiddityManager::make_property_subscriber (std::string subscriber_name,
				    void (*callback)(std::string subscriber_name,
						     std::string quiddity_name,
						     std::string property_name,
						     std::string value,
						     void *user_data),
				    void *user_data)
  {
     command_lock ();
     command_->set_name (QuiddityCommand::make_property_subscriber);
     command_->add_arg (subscriber_name);
     bool res = manager_impl_->make_property_subscriber (subscriber_name, callback, user_data);
     if (res)
       command_->result_.push_back("true");
     else
       command_->result_.push_back("false");
     command_unlock ();

     return res;
  }

  bool 
  QuiddityManager::remove_property_subscriber (std::string subscriber_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::remove_property_subscriber);
    command_->add_arg (subscriber_name);
    bool res = manager_impl_->remove_property_subscriber (subscriber_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }
  
  bool 
  QuiddityManager::subscribe_property (std::string subscriber_name,
				       std::string quiddity_name,
				       std::string property_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::subscribe_property);
    command_->add_arg (subscriber_name);
    command_->add_arg (quiddity_name);
    command_->add_arg (property_name);
 
    bool res = manager_impl_->subscribe_property (subscriber_name, quiddity_name, property_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }
  
  bool 
  QuiddityManager::unsubscribe_property (std::string subscriber_name,
					 std::string quiddity_name,
					 std::string property_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::unsubscribe_property);
    command_->add_arg (subscriber_name);
    command_->add_arg (quiddity_name);
    command_->add_arg (property_name);
     bool res = manager_impl_->unsubscribe_property (subscriber_name, quiddity_name, property_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }
  
  std::vector<std::string> 
  QuiddityManager::list_property_subscribers ()
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_property_subscribers);
    std::vector<std::string> res = manager_impl_->list_property_subscribers ();
    command_->result_ = res;
    command_unlock ();
    return res;
  }
  
  std::vector<std::pair<std::string, std::string> > 
  QuiddityManager::list_subscribed_properties (std::string subscriber_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_subscribed_properties);
    std::vector<std::pair<std::string, std::string> > res = manager_impl_->list_subscribed_properties (subscriber_name);
    //FIXME no result...
    command_unlock ();
    return res;
  }
  
  std::string 
  QuiddityManager::list_property_subscribers_json ()
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_property_subscribers_json);
    std::string res = manager_impl_->list_property_subscribers_json ();
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string 
  QuiddityManager::list_subscribed_properties_json (std::string subscriber_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_subscribed_properties_json);
    command_->add_arg (subscriber_name);
    std::string res = manager_impl_->list_subscribed_properties_json (subscriber_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }
  
  //lower level subscription
  bool
  QuiddityManager::subscribe_property_glib (std::string quiddity_name,
					    std::string property_name,
					    Property::Callback cb, 
					    void *user_data)
  {
    return manager_impl_->subscribe_property_glib (quiddity_name,
						   property_name,
						   cb,
						   user_data);
  }

  bool
  QuiddityManager::unsubscribe_property_glib (std::string quiddity_name,
					      std::string property_name,
					      Property::Callback cb, 
					      void *user_data)
  {
    return manager_impl_->unsubscribe_property_glib (quiddity_name,
						     property_name,
						     cb, 
						     user_data);
  }

  
  bool 
  QuiddityManager::invoke_va (const gchar *quiddity_name, 
			      ...)
  {
    std::vector<std::string> method_args;
    
    if (quiddity_name == NULL)
      {
	g_warning ("trying to invoke with a NULL quiddity name");
	return false;
      }
    va_list vl;
    va_start(vl, quiddity_name);
    char *method_name = va_arg(vl, char *);
    if (method_name == NULL)
      {
	g_warning ("trying to invoke with a NULL method name");
	return false;
      }
    char *method_arg = va_arg(vl, char *);
    while (method_arg != NULL)
      {
	method_args.push_back (method_arg);
	method_arg = va_arg(vl, char *);
      }
    va_end(vl);
    return invoke (quiddity_name, method_name, method_args);
  }

  bool 
  QuiddityManager::invoke (std::string quiddity_name, 
			   std::string method_name,
			   std::vector<std::string> args)
  {
    std::string res;
    command_lock ();
    command_->set_name (QuiddityCommand::invoke);
    command_->add_arg (quiddity_name);
    command_->add_arg (method_name);
    command_->set_vector_arg (args);
    invoke_in_gmainloop  ();
    res = command_->result_[0];
    command_unlock ();
    
    if (res == "true")
      return true;
    else 
      return false;
  } 
  
  bool
  QuiddityManager::auto_invoke (std::string method_name,
				std::vector<std::string> args)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::auto_invoke);
    command_->add_arg (method_name);
    command_->set_vector_arg (args);
    auto_invoke_method_name_ = method_name;
    auto_invoke_args_ = args;
    command_->result_.push_back ("true");
    command_unlock ();
   
    return true;
  }

  std::string
  QuiddityManager::get_methods_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_methods_description, 
		       quiddity_name.c_str(),
		       NULL);
  }

  std::string
  QuiddityManager::get_method_description (std::string quiddity_name, std::string method_name)
  {
    return seq_invoke (QuiddityCommand::get_method_description, 
		       quiddity_name.c_str(),
		       method_name.c_str(),
		       NULL);
  }

  std::string
  QuiddityManager::get_methods_description_by_class (std::string class_name)
  {
    return seq_invoke (QuiddityCommand::get_methods_description_by_class, 
		       class_name.c_str(),
		       NULL);
  }

  std::string
  QuiddityManager::get_method_description_by_class (std::string class_name, 
						    std::string method_name)
  {
    return seq_invoke (QuiddityCommand::get_method_description_by_class, 
		       class_name.c_str(),
		       method_name.c_str(),
		       NULL);
  }

  bool
  QuiddityManager::has_method (const std::string quiddity_name,
			       const std::string method_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::has_method);
    command_->add_arg (quiddity_name);
    command_->add_arg (method_name);
    bool res = manager_impl_->has_method (quiddity_name, method_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }

  bool 
  QuiddityManager::make_signal_subscriber (std::string subscriber_name,
					   void (*callback)(std::string subscriber_name,
							    std::string quiddity_name,
							    std::string property_name,
							    std::vector<std::string> params,
							    void *user_data),
					   void *user_data)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::make_signal_subscriber);
    command_->add_arg (subscriber_name);
     bool res = manager_impl_->make_signal_subscriber (subscriber_name, callback, user_data);
     if (res)
       command_->result_.push_back("true");
     else
       command_->result_.push_back("false");
     command_unlock ();

     return res;
  }

    bool 
QuiddityManager::remove_signal_subscriber (std::string subscriber_name)
    {
      command_lock ();
      command_->set_name (QuiddityCommand::remove_signal_subscriber);
      command_->add_arg (subscriber_name);
      bool res = manager_impl_->remove_signal_subscriber (subscriber_name);
      if (res)
	command_->result_.push_back("true");
      else
	command_->result_.push_back("false");
      command_unlock ();
      return res;
    }
  
  bool 
  QuiddityManager::subscribe_signal (std::string subscriber_name,
				     std::string quiddity_name,
				     std::string signal_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::subscribe_signal);
    command_->add_arg (subscriber_name);
    command_->add_arg (quiddity_name);
    command_->add_arg (signal_name);
 
    bool res = manager_impl_->subscribe_signal (subscriber_name, quiddity_name, signal_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }
  
  bool 
  QuiddityManager::unsubscribe_signal (std::string subscriber_name,
				       std::string quiddity_name,
				       std::string signal_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::unsubscribe_signal);
    command_->add_arg (subscriber_name);
    command_->add_arg (quiddity_name);
    command_->add_arg (signal_name);
    bool res = manager_impl_->unsubscribe_signal (subscriber_name, quiddity_name, signal_name);
    if (res)
      command_->result_.push_back("true");
    else
      command_->result_.push_back("false");
    command_unlock ();
    return res;
  }
  
  std::vector<std::string> 
  QuiddityManager::list_signal_subscribers ()
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_signal_subscribers);
    std::vector<std::string> res = manager_impl_->list_signal_subscribers ();
    command_->result_ = res;
    command_unlock ();
    return res;
  }
  
  std::vector<std::pair<std::string, std::string> > 
  QuiddityManager::list_subscribed_signals (std::string subscriber_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_subscribed_signals);
    std::vector<std::pair<std::string, std::string> > res = manager_impl_->list_subscribed_signals (subscriber_name);
    //FIXME no result...
    command_unlock ();
    return res;
  }
  
  std::string 
  QuiddityManager::list_signal_subscribers_json ()
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_signal_subscribers_json);
    std::string res = manager_impl_->list_signal_subscribers_json ();
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string 
  QuiddityManager::list_subscribed_signals_json (std::string subscriber_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::list_subscribed_signals_json);
    command_->add_arg (subscriber_name);
    std::string res = manager_impl_->list_subscribed_signals_json (subscriber_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string
  QuiddityManager::get_signals_description (std::string quiddity_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::get_signals_description);
    command_->add_arg (quiddity_name);
    std::string res = manager_impl_->get_signals_description (quiddity_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string
  QuiddityManager::get_signal_description (std::string quiddity_name, 
					   std::string signal_name)
  { 
    command_lock ();
    command_->set_name (QuiddityCommand::get_signal_description);
    command_->add_arg (quiddity_name);
    command_->add_arg (signal_name);
    std::string res = manager_impl_->get_signal_description (quiddity_name,
							     signal_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string
  QuiddityManager::get_signals_description_by_class (std::string class_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::get_signals_description_by_class);
    command_->add_arg (class_name);
    std::string res = manager_impl_->get_signals_description_by_class (class_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string
  QuiddityManager::get_signal_description_by_class (std::string class_name, 
						    std::string signal_name)
  {
    command_lock ();
    command_->set_name (QuiddityCommand::get_signal_description_by_class);
    command_->add_arg (class_name);
    command_->add_arg (signal_name);
    std::string res = manager_impl_->get_signal_description_by_class (class_name,
								      signal_name);
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  void 
  QuiddityManager::auto_init (std::string quiddity_name)
  {
    if (!manager_impl_->exists (quiddity_name))
      return;
    Quiddity::ptr quidd = manager_impl_->get_quiddity (quiddity_name);
    QuiddityManagerWrapper::ptr wrapper = std::dynamic_pointer_cast<QuiddityManagerWrapper> (quidd);
    if (wrapper)
      wrapper->set_quiddity_manager (shared_from_this());
    
    if (!auto_invoke_method_name_.empty ())
	if (quidd->has_method (auto_invoke_method_name_))
	  invoke (quidd->get_nick_name (), auto_invoke_method_name_,auto_invoke_args_);
  }
  
  std::string
  QuiddityManager::create (std::string quiddity_class)
  {
    std::string res = seq_invoke (QuiddityCommand::create, 
				  quiddity_class.c_str(),
				  NULL);
    auto_init (res);
    return res;
  }

  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    std::string res= seq_invoke (QuiddityCommand::create_nick_named, 
				 quiddity_class.c_str(), 
				 nick_name.c_str(), 
				 NULL);
    auto_init (res);
    return res;
  }

  bool
  QuiddityManager::remove (std::string quiddity_name)
  {
    
    std::string res = seq_invoke (QuiddityCommand::remove, 
				  quiddity_name.c_str(),
				  NULL);
    if (res == "true")
      return true;
    else
      return false;
  }

  std::vector<std::string> 
  QuiddityManager::get_classes ()
  {
    std::vector<std::string> res;
    command_lock ();
    command_->set_name (QuiddityCommand::get_classes);
    invoke_in_gmainloop  ();
    res = command_->result_;
    command_unlock ();
    return res;
  }

  std::string 
  QuiddityManager::get_classes_doc ()
  {
    return seq_invoke (QuiddityCommand::get_classes_doc,
		       NULL);
  }

  
  std::string 
  QuiddityManager::get_class_doc (std::string class_name)
  {
    return seq_invoke (QuiddityCommand::get_class_doc,
		       class_name.c_str (),
		       NULL);
  }

  std::string 
  QuiddityManager::get_quiddities_description ()
  {
    return seq_invoke (QuiddityCommand::get_quiddities_description,
		       NULL);
    //    return manager_impl_->get_quiddities_description ();
  }

  std::string 
  QuiddityManager::get_quiddity_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_quiddity_description,
		       quiddity_name.c_str (),
		       NULL);
    //return manager_impl_->get_quiddity_description (quiddity_name);
  }

 
  std::vector<std::string> 
  QuiddityManager::get_quiddities ()
  {
    std::vector<std::string> res;
    command_lock ();
    command_->set_name (QuiddityCommand::get_quiddities);
    invoke_in_gmainloop  ();
    res = command_->result_;
    command_unlock ();
    return res;
  }
  
  void
  QuiddityManager::init_command_sync ()
  {
    exec_cond_ = g_cond_new ();
    exec_mutex_ = g_mutex_new ();
    seq_mutex_ = g_mutex_new ();
  }

  void
  QuiddityManager::clear_command_sync ()
  {
    //FIXME may get the mutex before exiting 
  }

  //works for char * args only. Use NULL sentinel
  std::string
  QuiddityManager::seq_invoke (QuiddityCommand::command command, ...)
  {
    std::string res;
    command_lock ();
    va_list vl;
    va_start(vl, command);
    command_->set_name (command);
    char *command_arg = va_arg( vl, char *);
    while (command_arg != NULL)
      {
	command_->add_arg (command_arg);
	command_arg = va_arg( vl, char *);
      }
    va_end(vl);
    invoke_in_gmainloop ();
    res = command_->result_[0];
    command_unlock ();
    return res;
  }

  void
  QuiddityManager::invoke_in_gmainloop ()
  {
    GstUtils::g_idle_add_full_with_context (manager_impl_->get_g_main_context(),
					    G_PRIORITY_DEFAULT_IDLE,
					    GSourceFunc (execute_command),
					    this,
					    NULL);
    g_cond_wait (exec_cond_, exec_mutex_);
  }

  gboolean
  QuiddityManager::execute_command (gpointer user_data)
  {
    QuiddityManager *context = static_cast<QuiddityManager *>(user_data);

    
    g_mutex_lock (context->exec_mutex_);
    switch (context->command_->name_)
      {
      case QuiddityCommand::get_classes:
	context->command_->result_ = context->manager_impl_->get_classes ();
	break;
      case QuiddityCommand::get_classes_doc:
	context->command_->result_.push_back(context->manager_impl_->get_classes_doc ());
	break;
      case QuiddityCommand::get_class_doc:
	context->command_->result_.push_back(context->manager_impl_->get_class_doc (context->command_->args_[0]));
	break;
      case QuiddityCommand::get_quiddities:
	context->command_->result_ = context->manager_impl_->get_instances ();
	break;
      case QuiddityCommand::get_quiddities_description:
	context->command_->result_.push_back (context->manager_impl_->get_quiddities_description ());
	break;
      case QuiddityCommand::get_quiddity_description:
	context->command_->result_.push_back (context->manager_impl_->get_quiddity_description (context->command_->args_[0]));
	break;
      case QuiddityCommand::create:
	context->command_->result_.push_back (context->manager_impl_->create (context->command_->args_[0]));
	break;
      case QuiddityCommand::create_nick_named:
	context->command_->result_.push_back (context->manager_impl_->create (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::remove:
	if (context->manager_impl_->remove (context->command_->args_[0]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::get_properties_description:
	context->command_->result_.push_back (context->manager_impl_->get_properties_description (context->command_->args_[0]));
	break;
      case QuiddityCommand::get_property_description:
	context->command_->result_.push_back (context->manager_impl_->get_property_description (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::get_properties_description_by_class:
	context->command_->result_.push_back (context->manager_impl_->get_properties_description_by_class (context->command_->args_[0]));
	break;
      case QuiddityCommand::get_property_description_by_class:
	context->command_->result_.push_back (context->manager_impl_->get_property_description_by_class (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::set_property:
	if (context->manager_impl_->set_property (context->command_->args_[0], context->command_->args_[1], context->command_->args_[2]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::get_property:
	context->command_->result_.push_back (context->manager_impl_->get_property (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::get_methods_description:
	context->command_->result_.push_back (context->manager_impl_->get_methods_description (context->command_->args_[0]));
	break;
      case QuiddityCommand::get_method_description:
	context->command_->result_.push_back (context->manager_impl_->get_method_description (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::get_methods_description_by_class:
	context->command_->result_.push_back (context->manager_impl_->get_methods_description_by_class (context->command_->args_[0]));
	break;
      case QuiddityCommand::get_method_description_by_class:
	context->command_->result_.push_back (context->manager_impl_->get_method_description_by_class (context->command_->args_[0], context->command_->args_[1]));
	break;
      case QuiddityCommand::invoke:
	if (context->manager_impl_->invoke (context->command_->args_[0], context->command_->args_[1], context->command_->vector_arg_))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      default:
	g_error("unknown command");
      }
    g_cond_signal (context->exec_cond_);
    g_mutex_unlock (context->exec_mutex_);

    return FALSE; //remove from gmainloop
  }
  
  }
