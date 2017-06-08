/**
 * This is basically a Fibonacci heap with an added fast store for retrieving
 * nodes, and decrease their key as needed. Useful for search algorithms (e.g.
 * Dijstra, heuristic, ...).
 */

#ifndef FIBOQUEUE_H
#define FIBOQUEUE_H

#include "fiboheap.h"
#include <unordered_map>
#include <algorithm>

template<class T>
class FibQueue : public FibHeap<T>
{
 public:
  FibQueue()
    :FibHeap<T>()
    {
    }

  ~FibQueue()
    {
    }

  void decrease_key(typename FibHeap<T>::FibNode *x, int k)
  {
    typename std::unordered_map<T,typename FibHeap<T>::FibNode*>::iterator mit
      = find(x->key);
    fstore.erase(mit);
    fstore.insert(std::pair<T,typename FibHeap<T>::FibNode*>(k,x));
    FibHeap<T>::decrease_key(x,k);
  }
  
  typename FibHeap<T>::FibNode* push(T k, void *pl)
  {
    typename FibHeap<T>::FibNode *x = FibHeap<T>::push(k,pl);
    fstore.insert(std::pair<T,typename FibHeap<T>::FibNode*>(k,x));
    return x;
  }

  typename FibHeap<T>::FibNode* push(T k)
  {
    return push(k,NULL);
  }

  typename std::unordered_map<T,typename FibHeap<T>::FibNode*>::iterator find(T k)
  {
    typename std::unordered_map<T,typename FibHeap<T>::FibNode*>::iterator mit
      =fstore.find(k);
    return mit;
  }

  typename FibHeap<T>::FibNode* findNode(T k)
  {
    typename std::unordered_map<T,typename FibHeap<T>::FibNode*>::iterator mit
      = find(k);
    return (*mit).second;
  }
  
  void pop()
  {
    if (FibHeap<T>::empty())
      return;
    typename FibHeap<T>::FibNode *x = FibHeap<T>::extract_min();
    if (!x)
      return; // should not happen.
    auto range = fstore.equal_range(x->key);
    auto mit = std::find_if(range.first, range.second,
                            [x](const std::pair<T,typename FibHeap<T>::FibNode*> &ele){
                                return ele.second == x;
                            }
    );
    if (mit != range.second)
      fstore.erase(mit);
    else std::cerr << "[Error]: key " << x->key << " cannot be found in FiboQueue fast store\n";
    delete x;
  }
  
  std::unordered_multimap<T,typename FibHeap<T>::FibNode*> fstore;
};

#endif