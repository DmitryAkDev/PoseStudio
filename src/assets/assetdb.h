/**
 * @file assetdb.h
 * @brief Every Collections/Favorites SQL statement the Asset Manager runs, in one place.
 *
 * The Asset Manager keeps two kinds of virtual, database-backed sources: nested Collections
 * (AssetCollections rows, AssetCollectionParentID = 0 at the top level, each holding a flat
 * list of asset paths in AssetCollectionItems with a manual AssetCollectionItemSortOrder) and
 * the flat Favorites list (Favorites rows, FavoritePath unique, FavoriteSortOrder). Before
 * this header existed the statements were spread across the widget's context menus, drop
 * handlers and tree builders, several unchecked. Everything here returns success explicitly
 * and logs its own failure, so callers can keep the UI honest (A5: a drag-move never removes
 * an asset's only filing when the add half failed, because the move is one transaction here).
 * Library folders are NOT this module's business: those go through core/assetlibraries.h.
 *
 * Uses the shared application connection (appDatabase()); GUI thread only like the rest of the
 * data layer. QtSql only, no widgets.
 */

#ifndef ASSETDB_H
#define ASSETDB_H

#include <QHash>
#include <QList>
#include <QString>
#include <QStringList>

namespace AssetDb {

/// One AssetCollections row.
struct CollectionRow {
    int id = 0;
    QString name;
    int parentId = 0; ///< 0 = top level
};

/// Every collection, ordered by name (COLLATE NOCASE, the order the tree shows siblings in).
/// One query for the whole hierarchy: the caller nests them in memory.
QList<CollectionRow> allCollections();

/// `baseName` if no sibling under `parentCollectionId` already has it, otherwise
/// "baseName (2)", "baseName (3)", ... so repeated "New Collection" clicks under the same
/// parent each create a distinct collection instead of colliding on name.
QString uniqueCollectionName(const QString& baseName, int parentCollectionId);

/// Inserts a new collection under `parentCollectionId` (0 = top level). Returns its id, or -1.
int createCollection(const QString& name, int parentCollectionId);

bool renameCollection(int collectionId, const QString& newName);

/// The stored name of a collection (empty if it no longer exists): what an inline rename
/// reverts to when the new name is refused or the update fails.
QString collectionName(int collectionId);

/// Deletes the collection AND its items in one transaction (the tree row must only go when
/// this returns true). Sub-collections are the caller's concern: the UI only offers Delete
/// for collections that are empty (see collectionIsEmpty).
bool deleteCollection(int collectionId);

/// Re-parents one collection; descendants keep referencing its id, so they move with it.
bool reparentCollection(int collectionId, int newParentId);

/// True when the collection holds neither asset items nor sub-collections.
bool collectionIsEmpty(int collectionId);

/// Every collection's item paths, keyed by collection id: the one aggregate query behind the
/// tree's "does this collection hold an existing asset" icon state.
QHash<int, QStringList> collectionItemPathsByCollection();

/// A collection's item paths in the user's manual order (sort order, then insertion order).
QStringList collectionItemPaths(int collectionId);

/// Every favorited path in the user's manual order (sort order, then insertion order).
QStringList favoritePaths();

/// Files `path` into a collection, appended after the current max sort order.
bool addCollectionItem(const QString& path, int collectionId);
bool removeCollectionItem(const QString& path, int collectionId);

/// Adds `path` to Favorites (appended; a no-op if already favorited).
bool addFavorite(const QString& path);
bool removeFavorite(const QString& path);

/// Files many paths into a collection in one transaction (the "folder as collection" fill).
bool addCollectionItems(int collectionId, const QStringList& paths);

/// Where an asset is filed: Favorites, or a collection by id. Used to describe both halves of
/// a move.
struct Filing {
    bool favorites = false;
    int collectionId = -1;
};

/// Moves `path` from one filing to another ATOMICALLY: the remove and the add succeed or fail
/// together, so a failed add can never delete the asset's only filing.
bool moveItem(const QString& path, const Filing& from, const Filing& to);

/// Rewrites FavoriteSortOrder to match `pathsInOrder` (index = order), in one transaction.
bool setFavoriteOrder(const QStringList& pathsInOrder);

/// Rewrites a collection's AssetCollectionItemSortOrder to match `pathsInOrder`, in one
/// transaction.
bool setCollectionOrder(int collectionId, const QStringList& pathsInOrder);

} // namespace AssetDb

#endif // ASSETDB_H
