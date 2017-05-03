#include <iostream>
#include <list>
#include <map>

#include <Eigen/Dense>

#include <openmht/multi/MHT.h>
#include <openmht/plot/Plot.h>

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/uniform.hpp>
#include <boost/random/normal_distribution.hpp>

using std::cout;
using std::endl;

class Contact {
public:
     int id;
     Eigen::Vector2d position;
     Eigen::Vector2d velocity;
};


// Generate random numbers with normal dist
boost::mt19937 gener_;
boost::normal_distribution<> normal_dist_(0,1.0);
boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > rng_normal(gener_,normal_dist_);

void step_dynamics(std::list<Contact> &contacts, double dt)
{
     for (std::list<Contact>::iterator it = contacts.begin(); 
          it != contacts.end(); it++) {
          it->position = it->position + it->velocity*dt;
     }
}

Contact add_noise(Contact &contact)
{
     Contact c = contact;          
     c.position(0,0) = c.position(0,0) + rng_normal();
     c.position(1,0) = c.position(1,0) + rng_normal();          
     return c;
}

int main(int argc, char *argv[])
{


///////////////////////////////////////////////////////////////////////////////////////////////
// Generate fake data 

		//generating rand dists seeded on time 
     rng_normal.engine().seed(static_cast<unsigned int>(std::time(0)));
     rng_normal.distribution().reset();
     

     // For example data 
     int num_contacts = 3;   
     std::list<Contact> contacts;


     //Generating random readings 
     for (int i = 0; i < num_contacts; i++) {
          Contact c;     
          c.id = i;
          c.position << rng_normal()*10, rng_normal()*10;
          c.velocity << rng_normal()*1, rng_normal()*1;
          contacts.push_back(c);
     }

     // Printing rands with norm dist 
     for (int i = 0; i < 100; i++) {
          cout << rng_normal() << endl;
     }
     
///////////////////////////////////////////////////////////////////////////////////////////////
// Actual code starts 



     // TODO modify this to work continously using messages? 
     double t0 = 0; // start time
     double dt = 1; //update period 
     double tend = 20; //time end 
     
     openmht::Plot plot;
     std::map<int, std::list<Eigen::Vector2d> > truth, measured, tracked;
          
     //  start an instance of openMHT
     openmht::MHT mht;   
     // set the sensor update period (seconds).
     mht.set_dt(dt);


     for (double t = t0; t < tend; t += dt) {

          // Convert from your list of 2D detection points to a list of openMHT measurements 
          std::list<openmht::Measurement> m_list;


          // Add all new points to the list to process them (after for loop)
          for (std::list<Contact>::iterator it = contacts.begin(); it != contacts.end(); it++) {

          	   //latest measurement from sensor? 
               Contact c = add_noise(*it);
               
               //add measurement to list 
               openmht::Measurement m;
               m.set_position(c.position);
               m_list.push_back(m);

               // Save the measured position for plotting later 
               //(this uses the id, otherwise algo doesnt seem to)
               measured[c.id].push_back(c.position);
          }
          
          // Process the list of measurements with MHT
          // Runs algos and checks for culled tracks and if needed to be declared dead 
          mht.process_measurements(m_list);

          // Get a list of the fused entities / contacts:
          std::list<openmht::Entity> ents = mht.entities();

          // Save the filtered locations of each track
          for (std::list<openmht::Entity>::iterator it = ents.begin(); it != ents.end(); it++) {            
               tracked[it->id()].push_back(it->position());
          }                              
          
          // Save the truth tracks:
          for (std::list<Contact>::iterator it = contacts.begin(); it != contacts.end(); it++) {
               truth[it->id].push_back(it->position);
          }

          // Plot the results:
          std::string title = "Tracks";
          std::list< std::map<int, std::list<Eigen::Vector2d> > > lists;
          std::list<std::string> labels;
          std::list<std::string> styles;
          std::string options = "";
          std::list<std::string> objects;
          
          lists.push_back(truth);
          labels.push_back("Truth");
          styles.push_back("linespoints");
          
          lists.push_back(measured);
          labels.push_back("Measured");
          styles.push_back("linespoints");
          
          lists.push_back(tracked);          
          labels.push_back("Tracked");
          styles.push_back("linespoints");

          plot.plot(lists, title, labels, styles, options, objects, false);          

          // Step dynamics:
          step_dynamics(contacts,dt);
     }
     plot.wait();
}
