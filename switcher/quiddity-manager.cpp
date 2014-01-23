/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "quiddity-manager.h"
#include "quiddity.h" 
#include "gst-utils.h"
#include <string.h>
#include <iostream> //to remove

namespace switcher
{

  QuiddityManager::ptr 
  QuiddityManager::make_manager (std::string name)
  {
    if (! gst_is_initialized ())
      gst_init (NULL,NULL);
    QuiddityManager::ptr manager(new QuiddityManager(name));
    return manager;
  }


  QuiddityManager::QuiddityManager(std::string name) :
    manager_impl_ (QuiddityManager_Impl::make_manager (name)),
    name_ (name),
    command_ (),
    seq_mutex_ (),
    command_queue_ (g_async_queue_new ()),
    invocation_thread_ (std::thread (&QuiddityManager::invocation_thread, this)),
    execution_done_cond_ (),
    execution_done_mutex_ (),
    command_history_ (),
    history_begin_time_ (0)
  {
    reset_command_history (false);
  }

  QuiddityManager::~QuiddityManager()
  {
    clear_command_sync ();
    invocation_thread_.join ();
  }

  std::string
  QuiddityManager::get_name()
  {
    return name_;
  }

  void 
  QuiddityManager::reboot ()
  {
    {
      QuiddityManager_Impl::ptr manager;
      manager_impl_.swap (manager);
   }
    reset_command_history (false);
    manager_impl_ = QuiddityManager_Impl::make_manager (name_);
  }

  void
  QuiddityManager::reset_command_history (bool remove_created_quiddities)
  {
    if (remove_created_quiddities)
      {
	for (auto &it: command_history_)
	  {
	    if (g_str_has_prefix (QuiddityCommand::get_string_from_id (it->id_),
				  "create"))
		remove (it->result_[0]);
	  }
      }
    history_begin_time_ = g_get_monotonic_time ();
    command_history_.clear ();
  }
  
  void 
  QuiddityManager::command_lock ()
  {
    seq_mutex_.lock ();
    command_.reset (new QuiddityCommand ());
    gint64 cur_time = g_get_monotonic_time ();
    command_->time_ = cur_time - history_begin_time_;

  }

  void 
  QuiddityManager::command_unlock ()
  {
    //command has been invoked with the return value
    //save the command
    command_history_.push_back (command_);

     // JSONBuilder::ptr builder;
     // builder.reset (new JSONBuilder ());
     // builder->reset ();
     // builder->begin_object ();
     // builder->set_member_name ("command");
     // builder->add_node_value (command_->get_json_root_node ());
     // builder->end_object ();
     // g_print ("%s\n", builder->get_string(true).c_str ());

    seq_mutex_.unlock ();
  }

  void
  QuiddityManager::play_command_history (QuiddityManager::CommandHistory histo,
					 QuiddityManager::PropCallbackMap *prop_cb_data,
					 QuiddityManager::SignalCallbackMap *sig_cb_data,
					 bool mute_existing_subscribers)
  {

    if (mute_existing_subscribers)
      {
	manager_impl_->mute_property_subscribers (true);
	manager_impl_->mute_signal_subscribers (true);
      }

    for (auto &it: histo)
      {
	if (it->id_ == QuiddityCommand::make_property_subscriber)
	  {
	    if (prop_cb_data != NULL)
	      {
		QuiddityManager::PropCallbackMap::iterator prop_it = prop_cb_data->find (it->args_[0]);
		if (prop_it != prop_cb_data->end ())
		  make_property_subscriber (it->args_[0], prop_it->second.first, prop_it->second.second);
	      }
	  }
	else if (it->id_ == QuiddityCommand::make_signal_subscriber)
	  {
	    if (sig_cb_data != NULL)
	      {
		QuiddityManager::SignalCallbackMap::iterator sig_it = sig_cb_data->find (it->args_[0]);
		if (sig_it != sig_cb_data->end ())
		  make_signal_subscriber (it->args_[0], sig_it->second.first, sig_it->second.second);
	      }
	  }
	else
	  {
	    command_lock ();
	    command_ = it;
	    
	    g_debug ("running command %s", QuiddityCommand::get_string_from_id (command_->id_));
	    invoke_in_thread ();
	    //TODO test result consistency
	    command_unlock ();
	    if (command_->id_ == QuiddityCommand::create 
		|| command_->id_ == QuiddityCommand::create_nick_named)
	      auto_init (command_->result_[0]);
	  }
      }

    if (mute_existing_subscribers)
      {
	manager_impl_->mute_signal_subscribers (false);
	manager_impl_->mute_property_subscribers (false);
      }
    
  }
  
  std::vector <std::string> 
  QuiddityManager::get_property_subscribers_names (QuiddityManager::CommandHistory histo)
  {
    std::vector<std::string> res;
    for (auto &it: histo)
      if (it->id_ == QuiddityCommand::make_property_subscriber)
	res.push_back (it->args_[0]);
    return res; 
  }

  std::vector <std::string> 
  QuiddityManager::get_signal_subscribers_names (QuiddityManager::CommandHistory histo)
  {
    std::vector<std::string> res;
    for (auto &it: histo)
      if (it->id_ == QuiddityCommand::make_signal_subscriber)
	res.push_back (it->args_[0]);
    return res; 
  }

  QuiddityManager::CommandHistory 
  QuiddityManager::get_command_history_from_file (const char *file_path)
  {

    std::vector<QuiddityCommand::ptr> res;
    JsonParser *parser = json_parser_new ();
    GError *error = NULL;
    json_parser_load_from_file (parser,
				file_path,
				&error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_object_unref(parser);
	g_error_free (error);
	return res;
      }
    
    JsonNode *root_node = json_parser_get_root (parser);
    JsonReader *reader = json_reader_new (root_node);
    if (!json_reader_read_member (reader, "history"))
      {
	g_object_unref(reader);
	g_object_unref(parser);
	g_debug ("QuiddityManager::replay_command_history, no \"history\" member found");
	return res;
      }

    gint num_elements = json_reader_count_elements (reader);
    int i;
    for (i = 0; i < num_elements; i++)
      {
	json_reader_read_element (reader, i);
	res.push_back (QuiddityCommand::parse_command_from_json_reader (reader));
      }
    json_reader_end_element (reader);
   
    g_object_unref(reader);
    g_object_unref(parser);
    return res;
  }

  bool
  QuiddityManager::save_command_history (const char *file_path)
  {
    
    GFile *file = g_file_new_for_commandline_arg (file_path);
 
    GError *error = NULL;
    GFileOutputStream *file_stream = g_file_replace (file,
						    NULL,
						    TRUE, //make backup
						    G_FILE_CREATE_NONE ,
						    NULL,
						    &error);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }

    JSONBuilder::ptr builder;
    builder.reset (new JSONBuilder ());
    builder->reset ();
    builder->begin_object ();
    builder->set_member_name ("history");
    builder->begin_array ();

    for (auto &it: command_history_)
	builder->add_node_value (it->get_json_root_node ());
    builder->end_array ();
    builder->end_object ();
    
    gchar *history = g_strdup (builder->get_string(true).c_str ());

    g_output_stream_write ((GOutputStream *)file_stream,
			   history,
			   sizeof (gchar) * strlen (history),
			   NULL,
			   &error);
    g_free (history);
    if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }
     
    g_output_stream_close ((GOutputStream *)file_stream,
					   NULL,
					   &error);
     if (error != NULL)
      {
	g_warning ("%s",error->message);
	g_error_free (error);
	return false;
      }

     g_object_unref(file_stream);
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
					     QuiddityManager::PropCallback callback,
					     void *user_data)
  {
     command_lock ();
     command_->set_id (QuiddityCommand::make_property_subscriber);
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
    command_->set_id (QuiddityCommand::remove_property_subscriber);
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
    std::string res = seq_invoke (QuiddityCommand::subscribe_property,
				  subscriber_name.c_str (),
				  quiddity_name.c_str (),
				  property_name.c_str (),
				  NULL);
    if (g_strcmp0 (res.c_str (), "true") == 0)
      return true;
    return false;

    // command_lock ();
    // command_->set_id (QuiddityCommand::subscribe_property);
    // command_->add_arg (subscriber_name);
    // command_->add_arg (quiddity_name);
    // command_->add_arg (property_name);
 
    // bool res = manager_impl_->subscribe_property (subscriber_name, quiddity_name, property_name);
    // if (res)
    //   command_->result_.push_back("true");
    // else
    //   command_->result_.push_back("false");
    // command_unlock ();
    // return res;
  }
  
  bool 
  QuiddityManager::unsubscribe_property (std::string subscriber_name,
					 std::string quiddity_name,
					 std::string property_name)
  {
    std::string res = seq_invoke (QuiddityCommand::unsubscribe_property,
				  subscriber_name.c_str (),
				  quiddity_name.c_str (),
				  property_name.c_str (),
				  NULL);
    if (g_strcmp0 (res.c_str (), "true") == 0)
      return true;
    return false;
    // command_lock ();
    // command_->set_id (QuiddityCommand::unsubscribe_property);
    // command_->add_arg (subscriber_name);
    // command_->add_arg (quiddity_name);
    // command_->add_arg (property_name);
    //  bool res = manager_impl_->unsubscribe_property (subscriber_name, quiddity_name, property_name);
    // if (res)
    //   command_->result_.push_back("true");
    // else
    //   command_->result_.push_back("false");
    // command_unlock ();
    // return res;
  }
  
  std::vector<std::string> 
  QuiddityManager::list_property_subscribers ()
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_property_subscribers);
    std::vector<std::string> res = manager_impl_->list_property_subscribers ();
    command_->result_ = res;
    command_unlock ();
    return res;
  }
  
  std::vector<std::pair<std::string, std::string> > 
  QuiddityManager::list_subscribed_properties (std::string subscriber_name)
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_subscribed_properties);
    std::vector<std::pair<std::string, std::string> > res = manager_impl_->list_subscribed_properties (subscriber_name);
    //FIXME no result...
    command_unlock ();
    return res;
  }
  
  std::string 
  QuiddityManager::list_property_subscribers_json ()
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_property_subscribers_json);
    std::string res = manager_impl_->list_property_subscribers_json ();
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string 
  QuiddityManager::list_subscribed_properties_json (std::string subscriber_name)
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_subscribed_properties_json);
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
			      const gchar *method_name,
			      std::string **return_value,
			      ...)
  {

    command_lock ();
    std::vector<std::string> method_args;
    command_->set_id (QuiddityCommand::invoke);
    
    //g_print ("------ invoke_va\n");
    if (quiddity_name == NULL)
      {
	g_warning ("trying to invoke with a NULL quiddity name");
	command_unlock ();
	return false;
      }
    if (method_name == NULL)
      {
	g_warning ("trying to invoke with a NULL method name");
	command_unlock ();
	return false;
      }

    va_list vl;
    va_start(vl, return_value);
    char *method_arg = va_arg(vl, char *);
    while (method_arg != NULL)
      {
	method_args.push_back (method_arg);
	method_arg = va_arg(vl, char *);
      }
    va_end(vl);
    
    command_->add_arg (quiddity_name);
    command_->add_arg (method_name);
    command_->set_vector_arg (method_args);
    invoke_in_thread  ();
    if (return_value != NULL && !command_->result_.empty ())
      *return_value = new std::string (command_->result_[0]);
    bool res = command_->success_;

    command_unlock ();
    return res;
  }

  bool 
  QuiddityManager::invoke (const std::string quiddity_name, 
			   const std::string method_name,
			   std::string **return_value,
			   const std::vector<std::string> args)
  {
    //std::string res;
    command_lock ();
    command_->set_id (QuiddityCommand::invoke);
    command_->add_arg (quiddity_name);
    command_->add_arg (method_name);
    command_->set_vector_arg (args);
    invoke_in_thread  ();
    if (return_value != NULL && !command_->result_.empty ())
      *return_value = new std::string (command_->result_[0]);
    bool res = command_->success_;
    command_unlock ();
    return res;
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
    //FIXME do not have this 
    command_lock ();
    command_->set_id (QuiddityCommand::has_method);
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
  QuiddityManager::has_property (const std::string quiddity_name,
				 const std::string property_name)
  {
    return manager_impl_->has_property (quiddity_name, property_name);
  }

  bool 
  QuiddityManager::make_signal_subscriber (std::string subscriber_name,
					   QuiddityManager::SignalCallback callback,
					   void *user_data)
  {
    command_lock ();
    command_->set_id (QuiddityCommand::make_signal_subscriber);
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
      command_->set_id (QuiddityCommand::remove_signal_subscriber);
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
    std::string res = seq_invoke (QuiddityCommand::subscribe_signal,
				  subscriber_name.c_str (),
				  quiddity_name.c_str (),
				  signal_name.c_str (),
				  NULL);
    if (g_strcmp0 (res.c_str (), "true") == 0)
      return true;
    return false;
  }
  
  bool 
  QuiddityManager::unsubscribe_signal (std::string subscriber_name,
				       std::string quiddity_name,
				       std::string signal_name)
  {
    std::string res = seq_invoke (QuiddityCommand::unsubscribe_signal,
				  subscriber_name.c_str (),
				  quiddity_name.c_str (),
				  signal_name.c_str (),
				  NULL);
    if (g_strcmp0 (res.c_str (), "true") == 0)
      return true;
    return false;
 

    // command_lock ();
    // command_->set_id (QuiddityCommand::unsubscribe_signal);
    // command_->add_arg (subscriber_name);
    // command_->add_arg (quiddity_name);
    // command_->add_arg (signal_name);

    // bool res = manager_impl_->unsubscribe_signal (subscriber_name, quiddity_name, signal_name);
    //  if (res)
    //    command_->result_.push_back("true");
    //  else
    //    command_->result_.push_back("false");
    
    // command_unlock ();
    // return res;
  }
  
  std::vector<std::string> 
  QuiddityManager::list_signal_subscribers ()
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_signal_subscribers);
    std::vector<std::string> res = manager_impl_->list_signal_subscribers ();
    command_->result_ = res;
    command_unlock ();
    return res;
  }
  
  std::vector<std::pair<std::string, std::string> > 
  QuiddityManager::list_subscribed_signals (std::string subscriber_name)
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_subscribed_signals);
    std::vector<std::pair<std::string, std::string> > res = manager_impl_->list_subscribed_signals (subscriber_name);
    //FIXME no result...
    command_unlock ();
    return res;
  }
  
  std::string 
  QuiddityManager::list_signal_subscribers_json ()
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_signal_subscribers_json);
    std::string res = manager_impl_->list_signal_subscribers_json ();
    command_->result_.push_back(res);
    command_unlock ();
    return res;
  }

  std::string 
  QuiddityManager::list_subscribed_signals_json (std::string subscriber_name)
  {
    command_lock ();
    command_->set_id (QuiddityCommand::list_subscribed_signals_json);
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
    command_->set_id (QuiddityCommand::get_signals_description);
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
    command_->set_id (QuiddityCommand::get_signal_description);
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
    command_->set_id (QuiddityCommand::get_signals_description_by_class);
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
    command_->set_id (QuiddityCommand::get_signal_description_by_class);
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
  }
  
  std::string
  QuiddityManager::create (std::string quiddity_class)
  {
    std::string res = seq_invoke (QuiddityCommand::create, 
				  quiddity_class.c_str(),
				  NULL);
    return res;
  }

  bool
  QuiddityManager::rename (std::string nick_name, std::string new_nick_name)
  {
    std::string res = seq_invoke (QuiddityCommand::rename, 
				  nick_name.c_str(),
				  new_nick_name.c_str (),
				  NULL);
    if (res == "true")
      return true;
    else
      return false;
  
  }
  
  bool
  QuiddityManager::scan_directory_for_plugins (std::string directory)
  {
    std::string res = seq_invoke (QuiddityCommand::scan_directory_for_plugins, 
				  directory.c_str(),
				  NULL);
    if (res == "true")
      return true;
    else
      return false;
  }


  std::string
  QuiddityManager::create (std::string quiddity_class, std::string nick_name)
  {
    std::string res = seq_invoke (QuiddityCommand::create_nick_named, 
				 quiddity_class.c_str(), 
				 nick_name.c_str(), 
				 NULL);
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
    command_->set_id (QuiddityCommand::get_classes);
    invoke_in_thread  ();
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
  }

  std::string 
  QuiddityManager::get_quiddity_description (std::string quiddity_name)
  {
    return seq_invoke (QuiddityCommand::get_quiddity_description,
		       quiddity_name.c_str (),
		       NULL);
  }

 
  std::vector<std::string> 
  QuiddityManager::get_quiddities ()
  {
    std::vector<std::string> res;
    command_lock ();
    command_->set_id (QuiddityCommand::get_quiddities);
    invoke_in_thread  ();
    res = command_->result_;
    command_unlock ();
    return res;
  }
  
  void
  QuiddityManager::init_command_sync ()
  {
    //command_queue_ = g_async_queue_new (); //FIXME release that
    //invocation_thread_ (&invocation_thread, this);
//invocation_thread_ = g_thread_new ("invocation_thread", GThreadFunc(invocation_thread), this);
  }

  void
  QuiddityManager::invocation_thread ()
  {
    bool loop = true;
    while (loop)
      {
	 g_async_queue_pop (command_queue_);
	 if (command_->id_ != QuiddityCommand::quit)
	   execute_command (this);
	 else
	   loop = false;
	 {
	   std::unique_lock <std::mutex> lock (execution_done_mutex_);
	   execution_done_cond_.notify_all ();
	 }
      }
  }

  void
  QuiddityManager::clear_command_sync ()
  {
    command_lock ();
    command_->set_id (QuiddityCommand::quit);
    invoke_in_thread ();
    g_async_queue_unref (command_queue_);
    command_unlock ();
  }

  //works for char * args only. Use NULL sentinel
  std::string
  QuiddityManager::seq_invoke (QuiddityCommand::command command, ...)
  {
    std::string res;
    command_lock ();
    va_list vl;
    va_start(vl, command);
    command_->set_id (command);
    char *command_arg = va_arg( vl, char *);
    while (command_arg != NULL)
      {
	command_->add_arg (command_arg);
	command_arg = va_arg( vl, char *);
      }
    va_end(vl);

    // JSONBuilder::ptr builder;
    // builder.reset (new JSONBuilder ());
    // builder->reset ();
    // builder->begin_object ();
    // builder->set_member_name ("command");
    // builder->add_node_value (command_->get_json_root_node ());
    // builder->end_object ();
    // g_print ("%s\n", builder->get_string(true).c_str ());
    
    invoke_in_thread ();
    res = command_->result_[0];
    if (command_->id_ == QuiddityCommand::create 
	|| command_->id_ == QuiddityCommand::create_nick_named)
      auto_init (command_->result_[0]);
    command_unlock ();

    return res;
  }

  void
  QuiddityManager::invoke_in_thread ()
  {
    //g_print ("-- avant --\n");
    JSONBuilder::ptr builder;
    builder.reset (new JSONBuilder ());
    builder->reset ();
    builder->begin_object ();
    builder->set_member_name ("command");
    builder->add_node_value (command_->get_json_root_node ());
    builder->end_object ();
    //g_print ("%s\n", builder->get_string(true).c_str ());
    {
      std::unique_lock <std::mutex> lock (execution_done_mutex_);
      g_async_queue_push (command_queue_, command_.get ());
      execution_done_cond_.wait (lock);
    }
    // //FIXME remove that
    // timespec delay;
    // delay.tv_sec = 0;
    // delay.tv_nsec = 1000000;
    // nanosleep(&delay, NULL);

    //g_print ("-- apres --\n");
  }

  gboolean
  QuiddityManager::execute_command (gpointer user_data)
  {
    QuiddityManager *context = static_cast<QuiddityManager *>(user_data);
   
    switch (context->command_->id_)
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
      case QuiddityCommand::rename:
	if (context->manager_impl_->rename (context->command_->args_[0], context->command_->args_[1]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
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
	context->command_->result_.push_back (context->manager_impl_->get_method_description_by_class (context->command_->args_[0], 
												       context->command_->args_[1]));
	break;
      case QuiddityCommand::invoke:
	{
	  std::string *result;
	  if (context->manager_impl_->invoke (context->command_->args_[0],
					      context->command_->args_[1], 
					      &result,
					      context->command_->vector_arg_))
	    {
	      context->command_->success_ = true; //result_.push_back ("true");
	      context->command_->result_.push_back (*result);
	    }
	  else
	    context->command_->success_ = false; //result_.push_back ("false");

	  delete result;
	}
	break;
      case QuiddityCommand::subscribe_signal:
	if (context->manager_impl_->subscribe_signal (context->command_->args_[0], 
						      context->command_->args_[1], 
						      context->command_->args_[2]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::unsubscribe_signal:
	if (context->manager_impl_->unsubscribe_signal (context->command_->args_[0], 
							context->command_->args_[1], 
							context->command_->args_[2]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::scan_directory_for_plugins:
	if (context->manager_impl_->scan_directory_for_plugins (context->command_->args_[0].c_str ()))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::subscribe_property:
	if (context->manager_impl_->subscribe_property (context->command_->args_[0], 
							context->command_->args_[1], 
							context->command_->args_[2]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      case QuiddityCommand::unsubscribe_property:
	if (context->manager_impl_->unsubscribe_property (context->command_->args_[0], 
							  context->command_->args_[1], 
							  context->command_->args_[2]))
	  context->command_->result_.push_back ("true");
	else
	  context->command_->result_.push_back ("false");
	break;
      default:
	g_debug ("** unknown command, cannot launch %s\n", 
		 QuiddityCommand::get_string_from_id(context->command_->id_));
      }

    return FALSE; //remove from gmainloop
  }
  
}
