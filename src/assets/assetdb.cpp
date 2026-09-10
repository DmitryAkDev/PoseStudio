/**
 * @file assetdb.cpp
 * @brief Implementation of the AssetDb statements. See the header for what it is for.
 */

#include "assetdb.h"
#include "database.h"

#include <QDebug>
#include <QSet>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <functional>

namespace {

// Logs a failed statement with its context, returning false so callers can `return fail(...)`.
bool fail(const char* what, const QSqlQuery& q) {
    qWarning() << "[AssetDb]" << what << "failed:" << q.lastError().text();
    return false;
}

// Runs `body` inside one transaction: committed when it returns true, rolled back otherwise.
// Returns what body returned (false too when the commit itself fails).
bool inTransaction(const std::function<bool()>& body) {
    QSqlDatabase db = appDatabase();
    if (!db.transaction()) {
        qWarning() << "[AssetDb] could not begin transaction:" << db.lastError().text();
        return false;
    }
    if (!body()) {
        db.rollback();
        return false;
    }
    if (!db.commit()) {
        qWarning() << "[AssetDb] commit failed:" << db.lastError().text();
        db.rollback();
        return false;
    }
    return true;
}

// The non-transactional halves, shared by the public functions and by moveItem's transaction.

bool insertCollectionItem(const QString& path, int collectionId) {
    QSqlQuery q(appDatabase());
    // New items append to the end of the collection's manual order (max existing order + 1).
    q.prepare("INSERT INTO AssetCollectionItems (AssetCollectionItemPath, AssetCollectionItemCol, AssetCollectionItemSortOrder) "
              "VALUES (:path, :id, (SELECT COALESCE(MAX(AssetCollectionItemSortOrder), -1) + 1 FROM AssetCollectionItems WHERE AssetCollectionItemCol = :id2))");
    q.bindValue(":path", path);
    q.bindValue(":id", collectionId);
    q.bindValue(":id2", collectionId);
    return q.exec() || fail("add asset to collection", q);
}

bool deleteCollectionItem(const QString& path, int collectionId) {
    QSqlQuery q(appDatabase());
    q.prepare("DELETE FROM AssetCollectionItems WHERE AssetCollectionItemPath = :path AND AssetCollectionItemCol = :id");
    q.bindValue(":path", path);
    q.bindValue(":id", collectionId);
    return q.exec() || fail("remove asset from collection", q);
}

bool insertFavorite(const QString& path) {
    QSqlQuery q(appDatabase());
    // New favorites append to the end of the user's order (max existing order + 1).
    q.prepare("INSERT OR IGNORE INTO Favorites (FavoritePath, FavoriteSortOrder) "
              "VALUES (:path, (SELECT COALESCE(MAX(FavoriteSortOrder), -1) + 1 FROM Favorites))");
    q.bindValue(":path", path);
    return q.exec() || fail("add asset to favorites", q);
}

bool deleteFavorite(const QString& path) {
    QSqlQuery q(appDatabase());
    q.prepare("DELETE FROM Favorites WHERE FavoritePath = :path");
    q.bindValue(":path", path);
    return q.exec() || fail("remove asset from favorites", q);
}

bool applyFiling(const QString& path, const AssetDb::Filing& filing, bool add) {
    if (filing.favorites) return add ? insertFavorite(path) : deleteFavorite(path);
    if (filing.collectionId > 0)
        return add ? insertCollectionItem(path, filing.collectionId)
                   : deleteCollectionItem(path, filing.collectionId);
    return true; // no filing (a plain library/search view): nothing to do
}

} // namespace

namespace AssetDb {

QList<CollectionRow> allCollections() {
    QList<CollectionRow> rows;
    QSqlQuery q(appDatabase());
    if (!q.exec("SELECT AssetCollectionID, AssetCollectionName, AssetCollectionParentID FROM AssetCollections "
                "ORDER BY AssetCollectionName COLLATE NOCASE")) {
        fail("load collections", q);
        return rows;
    }
    while (q.next())
        rows.append({q.value(0).toInt(), q.value(1).toString(), q.value(2).toInt()});
    return rows;
}

QString uniqueCollectionName(const QString& baseName, int parentCollectionId) {
    QSqlQuery q(appDatabase());
    q.prepare("SELECT AssetCollectionName FROM AssetCollections WHERE AssetCollectionParentID = :pid");
    q.bindValue(":pid", parentCollectionId);
    if (!q.exec()) fail("list sibling collection names", q);

    QSet<QString> existingNames;
    while (q.next()) existingNames.insert(q.value(0).toString());

    if (!existingNames.contains(baseName)) return baseName;

    int suffix = 2;
    QString candidate;
    do {
        candidate = QStringLiteral("%1 (%2)").arg(baseName).arg(suffix++);
    } while (existingNames.contains(candidate));

    return candidate;
}

int createCollection(const QString& name, int parentCollectionId) {
    QSqlQuery ins(appDatabase());
    ins.prepare("INSERT INTO AssetCollections (AssetCollectionName, AssetCollectionParentID) VALUES (:name, :pid)");
    ins.bindValue(":name", name);
    ins.bindValue(":pid", parentCollectionId);
    if (!ins.exec()) {
        fail("create collection", ins);
        return -1;
    }
    return ins.lastInsertId().toInt();
}

bool renameCollection(int collectionId, const QString& newName) {
    QSqlQuery q(appDatabase());
    q.prepare("UPDATE AssetCollections SET AssetCollectionName = :name WHERE AssetCollectionID = :id");
    q.bindValue(":name", newName);
    q.bindValue(":id", collectionId);
    return q.exec() || fail("rename collection", q);
}

QString collectionName(int collectionId) {
    QSqlQuery q(appDatabase());
    q.prepare("SELECT AssetCollectionName FROM AssetCollections WHERE AssetCollectionID = :id");
    q.bindValue(":id", collectionId);
    if (!q.exec()) { fail("read collection name", q); return {}; }
    return q.next() ? q.value(0).toString() : QString();
}

bool deleteCollection(int collectionId) {
    return inTransaction([&]() {
        QSqlQuery q(appDatabase());
        q.prepare("DELETE FROM AssetCollections WHERE AssetCollectionID = :id");
        q.bindValue(":id", collectionId);
        if (!q.exec()) return fail("delete collection", q);

        QSqlQuery q2(appDatabase());
        q2.prepare("DELETE FROM AssetCollectionItems WHERE AssetCollectionItemCol = :id");
        q2.bindValue(":id", collectionId);
        if (!q2.exec()) return fail("delete collection items", q2);
        return true;
    });
}

bool reparentCollection(int collectionId, int newParentId) {
    QSqlQuery q(appDatabase());
    q.prepare("UPDATE AssetCollections SET AssetCollectionParentID = :pid WHERE AssetCollectionID = :id");
    q.bindValue(":pid", newParentId);
    q.bindValue(":id", collectionId);
    return q.exec() || fail("reparent collection", q);
}

bool collectionIsEmpty(int collectionId) {
    // Deletable only when both asset items and sub-collections are absent.
    QSqlQuery chk(appDatabase());
    chk.prepare("SELECT COUNT(*) FROM AssetCollectionItems WHERE AssetCollectionItemCol = :id");
    chk.bindValue(":id", collectionId);
    if (chk.exec() && chk.next() && chk.value(0).toInt() > 0) return false;

    QSqlQuery chk2(appDatabase());
    chk2.prepare("SELECT COUNT(*) FROM AssetCollections WHERE AssetCollectionParentID = :id");
    chk2.bindValue(":id", collectionId);
    if (chk2.exec() && chk2.next() && chk2.value(0).toInt() > 0) return false;
    return true;
}

QHash<int, QStringList> collectionItemPathsByCollection() {
    QHash<int, QStringList> byCollection;
    QSqlQuery q(appDatabase());
    if (!q.exec("SELECT AssetCollectionItemCol, AssetCollectionItemPath FROM AssetCollectionItems")) {
        fail("load collection items", q);
        return byCollection;
    }
    while (q.next())
        byCollection[q.value(0).toInt()].append(q.value(1).toString());
    return byCollection;
}

QStringList collectionItemPaths(int collectionId) {
    QSqlQuery q(appDatabase());
    // Preserve the user's manual drag order (AssetCollectionItemSortOrder), falling back to
    // insertion order.
    q.prepare("SELECT AssetCollectionItemPath FROM AssetCollectionItems WHERE AssetCollectionItemCol = :id "
              "ORDER BY AssetCollectionItemSortOrder ASC, AssetCollectionItemID ASC");
    q.bindValue(":id", collectionId);
    QStringList paths;
    if (!q.exec()) {
        fail("load collection items", q);
        return paths;
    }
    while (q.next()) paths.append(q.value(0).toString());
    return paths;
}

QStringList favoritePaths() {
    QSqlQuery q(appDatabase());
    QStringList paths;
    // Preserve the user's manual drag order (FavoriteSortOrder), falling back to insertion order.
    if (!q.exec("SELECT FavoritePath FROM Favorites ORDER BY FavoriteSortOrder ASC, FavoriteID ASC")) {
        fail("load favorites", q);
        return paths;
    }
    while (q.next()) paths.append(q.value(0).toString());
    return paths;
}

bool addCollectionItem(const QString& path, int collectionId) {
    return insertCollectionItem(path, collectionId);
}

bool removeCollectionItem(const QString& path, int collectionId) {
    return deleteCollectionItem(path, collectionId);
}

bool addFavorite(const QString& path) {
    return insertFavorite(path);
}

bool removeFavorite(const QString& path) {
    return deleteFavorite(path);
}

bool addCollectionItems(int collectionId, const QStringList& paths) {
    return inTransaction([&]() {
        QSqlQuery q(appDatabase());
        q.prepare("INSERT INTO AssetCollectionItems (AssetCollectionItemPath, AssetCollectionItemCol) VALUES (:path, :id)");
        for (const QString& path : paths) {
            q.bindValue(":path", path);
            q.bindValue(":id", collectionId);
            if (!q.exec()) return fail("add folder assets to collection", q);
        }
        return true;
    });
}

bool moveItem(const QString& path, const Filing& from, const Filing& to) {
    return inTransaction([&]() {
        return applyFiling(path, to, true) && applyFiling(path, from, false);
    });
}

bool setFavoriteOrder(const QStringList& pathsInOrder) {
    return inTransaction([&]() {
        QSqlQuery q(appDatabase());
        q.prepare("UPDATE Favorites SET FavoriteSortOrder = :ord WHERE FavoritePath = :path");
        for (int i = 0; i < pathsInOrder.size(); ++i) {
            q.bindValue(":ord", i);
            q.bindValue(":path", pathsInOrder[i]);
            if (!q.exec()) return fail("persist favorites order", q);
        }
        return true;
    });
}

bool setCollectionOrder(int collectionId, const QStringList& pathsInOrder) {
    return inTransaction([&]() {
        QSqlQuery q(appDatabase());
        q.prepare("UPDATE AssetCollectionItems SET AssetCollectionItemSortOrder = :ord "
                  "WHERE AssetCollectionItemPath = :path AND AssetCollectionItemCol = :id");
        for (int i = 0; i < pathsInOrder.size(); ++i) {
            q.bindValue(":ord", i);
            q.bindValue(":path", pathsInOrder[i]);
            q.bindValue(":id", collectionId);
            if (!q.exec()) return fail("persist collection order", q);
        }
        return true;
    });
}

} // namespace AssetDb
