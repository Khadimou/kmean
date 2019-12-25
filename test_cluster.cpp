#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <chrono>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct Point{
  double x,y;
};

// générer le jeu de données
inline auto make_uniform_dataset(int bound_min,int bound_max,int nombre_point)
  {
    auto x = std::vector<double>(nombre_point);
    auto y = std::vector<double>(nombre_point);

    auto uniform = std::uniform_real_distribution<double>(bound_min,bound_max);
    auto rg = std::default_random_engine();

    for(auto n=0;n<nombre_point;n++){
      x[n] = uniform(rg);
      y[n] = uniform(rg);
    }

    return std::pair{x,y};
  }

//calcul de la distance euclidienne entre 2 points
inline auto distance_euclid(Point A,Point B){
  return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y));
}

//tirer un nombre au hasard dans mon jeu de données
inline auto tirer_nombre(std::vector<double>& t){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0,t.size()-1);
  return t[dis(gen)];
}

//tirer un nombre entier au hasard dans mon jeu de données
inline auto tirer_indice(std::vector<int>& t){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(0,t.size()-1);
  return dis(gen);
}

//tirer un point au hasard dans mon jeu de données
inline auto tirer_point(std::vector<Point>& t){
  auto k = 0;
  std::vector<double> val_x(t.size());
  std::vector<double> val_y(t.size());
  for(int i=0;i<t.size();i++){
    val_x[i] = t[i].x;
    val_y[i] = t[i].y;
  }
  t[k].x = tirer_nombre(val_x);
  t[k].y = tirer_nombre(val_y);
  //std::cout << "centre[" << 0 << "]= " << t[k].x << std::endl;
  
  return std::pair{t[k].x,t[k].y};
}

inline auto kmean(std::vector<double>& x,std::vector<double>& y,int nbre_cluster,int iter){
  auto start = std::chrono::system_clock::now();
  std::vector<Point> A(2*x.size()); //on définit un point aléatoire
  //1.generer des centres aléatoires
  int k=0;
  std::vector<Point> center(nbre_cluster);
  std::vector<double> renvoi_x(nbre_cluster);
  std::vector<double> renvoi_y(nbre_cluster);
  //on stocke les valeurs de x et y dans A et on choisit des centres aléatoires parmi les valeurs de x et y
  for(int i=0;i<nbre_cluster;i++){
    for(k=0;k<A.size();k++){
    A[k].x = x[k];
    A[k].y = y[k+1];
    }
    center[i].x = tirer_point(A).first;
    center[i].y = tirer_point(A).second;
  }

  std::vector<int> stock(A.size());

  for(int i = 0;i < iter;i++){
  //2.boucle pour trouver proximité des points
  for(int s=0;s<A.size();s++){
    
    auto best_distance = std::numeric_limits<double>::max();
    auto best_cluster = 0;
    for(int i=0;i<nbre_cluster;i++){
     const auto distance = distance_euclid(A[s],center[i]);
    if(distance < best_distance){
      best_distance = distance;
      best_cluster = i;
    }
    }
    stock[s] = best_cluster;
  }

  std::vector<Point> new_center(nbre_cluster);

  std::vector<size_t> counts(k, 0);
  for(auto point = 0;point <A.size();point++){
    const auto cluster = stock[point];
    new_center[cluster].x += x[point];
    new_center[cluster].y += y[point];
    counts[cluster] += 1;
  }

  //diviser les sommes à chaque compte pour obtenir des nouveaux centres
  for(auto cluster=0;cluster<nbre_cluster;cluster++){
  
    const auto count = std::max<size_t>(1, counts[cluster]);
    center[cluster].x = new_center[cluster].x / count;
    center[cluster].y = new_center[cluster].y / count;
  }
    renvoi_x[i] = center[i].x;
    renvoi_y[i] = center[i].y;
}
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end-start;
  std::cout << "L'execution pour le k-mean prend " << diff.count() << " s\n" << std::endl;

  return std::pair{renvoi_x,renvoi_y};
}

auto makeblob(int nbre_point_blob,int nbre_blob,double bound_min,double bound_max,double centre_x,double centre_y){
   auto start = std::chrono::system_clock::now();
  auto blobx = std::vector<double>(nbre_point_blob*nbre_blob);
  auto bloby = std::vector<double>(nbre_point_blob*nbre_blob);

  std::default_random_engine gen;

  for(int i=bound_min;i<bound_max;i++){
    for(int c=0;c<nbre_blob;c++){
    std::normal_distribution <double> dis(centre_x,1);
    for(auto& j: blobx){
        j = dis(gen);
        blobx[c] = j;
        }
      }
      }

  for(int j=bound_min;j<bound_max;j++){
    for(int c=0;c<nbre_blob;c++){
      std::normal_distribution <double> dis(centre_y,1);
      for(auto& y: bloby){
          y = dis(gen);
          bloby[c] = y;
        }
      }
      }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end-start;
  std::cout << "L'execution pour la génération de points par blob prend " << diff.count() << " s\n" << std::endl;

  return std::pair{blobx,bloby};
}

int main(int argc,char *argv[]){
  if(argc < 2){
    std::cout << "Usage : [nombre de centres] missing " << std::endl;
    return 1;
  }
  int K = atoi(argv[1]);

  auto points = make_uniform_dataset(-10,10,100);
  std::vector<double> tab;
  for(int i=0;i<points.first.size();i++){
  tab.push_back(points.first[i]);
  tab.push_back(points.second[i]);
  }
  
  
  auto m = kmean(points.first,points.second,K,K); //ne pas dépasser K itérations

  std::vector<std::string> color_str = {"bo", "go", "ro", "ko", "yo"};

  std::vector<int> indice(color_str.size());
  for(int ind=0;ind<color_str.size();ind++){
    indice[ind] = ind;
  }

  for(int i=0;i<m.first.size();i++){
  auto g = makeblob(tab.size()/K,K,50,100,m.first[i],m.second[i]);
  int cc = tirer_indice(indice);
  plt::plot(g.first,g.second,color_str[cc]);
  }
  plt::plot(m.first,m.second,"o");
  plt::show();
  
  return 0;
}
