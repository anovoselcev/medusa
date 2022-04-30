#ifndef QUICK_HULL_HPP
#define QUICK_HULL_HPP

#include <glm/glm.hpp>
#include <boost/container_hash/hash.hpp>
#include <unordered_set>
#include <list>
#include <vector>

namespace medusa::alg{

   using FaceIndexed = std::tuple<size_t, size_t, size_t>;

   struct QuickHull{

      struct Edge{
         size_t p[2] = {0, 0};

         Edge(size_t p1, size_t p2) : p{std::min(p1, p2), std::max(p1, p2)}{
            assert(p1 != p2);
         }

         bool operator==(const Edge& e) const{
            return p[0] == e.p[0] && p[1] == e.p[1];
         }
      };

      struct EdgeHash{
         size_t operator()(const Edge& e) const{
            size_t seed = 0;
            boost::hash_combine(seed, e.p[0]);
            boost::hash_combine(seed, e.p[1]);
            return seed;
         }
      };

      struct Face{
         Edge e[3];
         size_t p[3] = {0, 0, 0};

         Face(size_t p1, size_t p2, size_t p3) : e{{p1, p2}, {p2, p3}, {p3, p1}},
          p{std::min(e[0].p[0], e[2].p[0]),
            std::min(std::min(e[0].p[1], e[1].p[1]), e[2].p[1]),
            std::max(e[0].p[1], e[2].p[1])}{
            
            assert(p1 != p2);
            assert(p2 != p3);
            assert(p3 != p1);
         }

         bool operator==(const Face& f) const{
            for(uint8_t i = 0; i < 3; ++i) if(f.p[i] != p[i]) return false;
            return true;
         }

         bool isAdjacent(const Edge& edge) const{
            return edge == e[0] || edge == e[1] || edge == e[2];
         }

         bool isAdjacent(const Face& f) const{
            return f.isAdjacent(e[0]) || f.isAdjacent(e[1]) || f.isAdjacent(e[2]);
         }
      };

      struct FaceHash{
         size_t operator()(const Face& f) const{
            size_t seed = 0;
            boost::hash_combine(seed, f.e[0].p[0]);
            boost::hash_combine(seed, f.e[0].p[1]);
            boost::hash_combine(seed, f.e[1].p[0]);
            boost::hash_combine(seed, f.e[1].p[1]);
            boost::hash_combine(seed, f.e[2].p[0]);
            boost::hash_combine(seed, f.e[2].p[1]);
            boost::hash_combine(seed, f.p[0]);
            boost::hash_combine(seed, f.p[1]);
            boost::hash_combine(seed, f.p[2]);
            return seed;
         }
      };

      using PointSet = std::unordered_set<size_t>;
      using FaceSet = std::unordered_set<Face, FaceHash>;
      using EdgeSet = std::unordered_set<Edge, EdgeHash>;

      struct FaceProccessUnit{
         Face face;
         PointSet views;
      };

      using FaceList = std::list<FaceProccessUnit>;

      struct Plane{
         glm::vec3 n;
         double d;
      };

      std::vector<glm::vec3> points_;
      FaceList faces_;
      size_t internalPts[4] = {0, 0, 0, 0};




      QuickHull(const std::vector<glm::vec3>& points) : points_(points){}

      Plane getPlane(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2){
         double plane[4] = {0, 0, 0, 0};
         plane[0] = p1.y * (p2.z - p0.z) + p0.y * (p1.z - p2.z) + p2.y * (p0.z - p1.z);
         plane[1] = p1.x * (p0.z - p2.z) + p0.x * (p2.z - p1.z) + p2.x * (p1.z - p0.z);
         plane[2] = p1.x * (p2.y - p0.y) + p0.x * (p1.y - p2.y) + p2.x * (p0.y - p1.y);
         plane[3] = -plane[0] * p0.x - plane[1] * p0.y - plane[2] * p0.z;
         glm::vec3 n = {plane[0], plane[1], plane[2]};
         plane[3] /= glm::length(n);
         Plane res;
         res.n = glm::normalize(n);
         res.d = plane[3];
         for(uint8_t i = 0; i < 4; ++i){
            auto pts = points_[internalPts[i]];
            auto dist = glm::dot(pts, res.n) + res.d;
            if(std::fabs(dist) > 0.1){
               if(dist > 0){
                  res.n.x *= -1;
                  res.n.y *= -1;
                  res.n.z *= -1;
                  res.d *= -1;
                  return res;
               }
            }
         }
         return res;
      }

      Plane getPlane(const Face& f){
         return getPlane(points_[f.p[0]], points_[f.p[1]], points_[f.p[2]]);
      }

      bool isFaceVisibleFromPoint(const Face& f, size_t view){
         auto pln = getPlane(f);
         auto vec = points_[view] - points_[f.p[0]];
         return glm::dot(pln.n, vec) > 0.0005;
      }

      void initialPhase(){
         size_t x[2] = {0, 0};
         size_t y[2] = {0, 0};
         size_t z[2] = {0, 0};

         for(size_t i = 0; i < points_.size(); ++i){
            if(points_[i].x < points_[x[0]].x) x[0] = i;
            if(points_[i].x > points_[x[1]].x) x[1] = i;
            if(points_[i].y < points_[y[0]].y) y[0] = i;
            if(points_[i].y > points_[y[1]].y) y[1] = i;
            if(points_[i].z < points_[z[0]].z) z[0] = i;
            if(points_[i].z > points_[z[1]].z) z[1] = i;
         }

         size_t line1[2] = {0, 0};
         size_t boundPoints[6] = {x[0], x[1], y[0], y[1], z[0], z[1]};
         for(uint8_t i = 0; i < 6; ++i){
            for(uint8_t j = i; j < 6; ++j){
               if(i != j){
                  auto maxDist = glm::length(points_[line1[0]] - points_[line1[1]]);
                  auto currDist = glm::length(points_[boundPoints[i]] - points_[boundPoints[j]]);
                  if(currDist > maxDist){
                     line1[0] = boundPoints[i];
                     line1[1] = boundPoints[j];
                  }
               }
            }
         }

         size_t line2[2] = {line1[0], 0};
         size_t line3[2] = {0, line1[1]};

         for(uint8_t i = 0; i < 6; ++i){
            auto s = points_[line1[0]] - points_[line1[1]];
            auto maxDist = glm::length(glm::cross((points_[line2[0]] - points_[line2[1]]), s)) / glm::length(s);
            auto currDist = glm::length(glm::cross((points_[line2[0]] - points_[boundPoints[i]]), s)) / glm::length(s);
            if(currDist > maxDist) line2[1] = boundPoints[i];
         }
         line3[0] = line2[1];


         auto plane = getPlane(points_[line1[0]], points_[line1[1]], points_[line2[1]]);
         size_t top = 0;
         for(uint8_t i = 0; i < 6; ++i){
            auto p = points_[top];
            auto maxDist = std::fabs(glm::dot(p, plane.n) + plane.d);
            p = points_[boundPoints[i]];
            auto currDist = std::fabs(glm::dot(p, plane.n) + plane.d);
            if(currDist > maxDist) top = boundPoints[i];
         }

         Face f1{line1[0], line1[1], line2[1]};
         Face f2{line1[0], line1[1], top};
         Face f3{line1[1], line2[1], top};
         Face f4{line2[1], line1[0], top};
         Face f[4] = {f1, f2, f3, f4};

         internalPts[0] = line1[0];
         internalPts[1] = line1[1];
         internalPts[2] = line2[1];
         internalPts[3] = top;
         
         for(uint8_t i = 0; i < 4; ++i){
            PointSet views;
            for(size_t j = 0; j < points_.size(); ++j)
               if(isFaceVisibleFromPoint(f[i], j)) views.insert(j);
            faces_.insert(faces_.begin(), {f[i], std::move(views)});
         }
      }

      bool calcHorizon(FaceSet& faces, EdgeSet& edges, const Face& face, size_t view){

         if(isFaceVisibleFromPoint(face, view)){
            faces.insert(face);
            for(uint8_t i = 0; i < 3; ++i){
               auto it = std::find_if(faces_.begin(), faces_.end(),
               [&face, i, &faces](const auto& punit){
                  return punit.face.isAdjacent(face.e[i]) && !faces.contains(punit.face); 
               });
               if(it != faces_.end()){
                  auto neighbour = it->face;
                  if(calcHorizon(faces, edges, neighbour, view)) edges.insert(face.e[i]);
               }
            }
            return false;
         }
         return true;
      }


      std::vector<FaceIndexed> make(){
         initialPhase();
         FaceSet finalFaces;
         std::vector<FaceIndexed> res;
         size_t maxFaces = 0;

        bool anyLeft = true;
        while(anyLeft){
           anyLeft = false;
           auto it = faces_.begin();
           while(it != faces_.end()){
              auto& face = it->face;
              auto& views = it->views;
              bool needToInc = true;
              if(!views.empty()){
                  anyLeft = true;
                  size_t top = *views.begin();
                  auto pln = getPlane(face);
                  for(auto view : views){
                     const auto& realPts = points_[top];
                     auto maxDist = std::fabs(glm::dot(realPts, pln.n) + pln.d);
                     const auto& newPts = points_[view];
                     auto currDist = std::fabs(glm::dot(newPts, pln.n) + pln.d);
                     if(currDist > maxDist) top = view; 
                  }

                  FaceSet faces;
                  EdgeSet edges;
                  calcHorizon(faces, edges, face, top);

                  for(auto& internalFace : faces){
                     auto foundedFace = std::find_if(faces_.begin(), faces_.end(),
                     [&internalFace](const auto& puint){
                        return puint.face == internalFace;
                     });
                     if(foundedFace != faces_.end()){
                        if(foundedFace == it){
                           it = faces_.erase(foundedFace);
                           needToInc = false;
                        }
                        else faces_.erase(foundedFace);
                     }
                  }

                  for(auto& edge : edges){
                     Face newFace(edge.p[0], edge.p[1], top);
                     PointSet newViews;
                     for(size_t j = 0; j < points_.size(); ++j)
                        if(isFaceVisibleFromPoint(newFace, j)) newViews.insert(j);
                     faces_.insert(faces_.end(), {newFace, std::move(newViews)});
                  }

                  if(faces_.size() > maxFaces) maxFaces = faces_.size();
               }
               else finalFaces.insert(face);
               if(needToInc) it++;
           }
        }

         for(const auto& face : finalFaces){
            res.push_back({face.p[0], face.p[1], face.p[2]});
         }
         return res;
      }
         
    };
}

#endif